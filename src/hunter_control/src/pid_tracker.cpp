#include "hunter_control/pid_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "hunter_control/hunter_control_common.hpp"

namespace hunter_control
{

namespace
{
// Starting gains from the Phase 3 plan's gain table -- override permanently
// via a params YAML (e.g. monza_defaults.yaml's pid_tracker.ros__parameters.
// steering_pid.p etc.), or tune live via `ros2 param set /pid_tracker
// steering_pid.p <value>` for one run. control_toolbox::PidROS::
// initialize_from_args() sets the actual PID gains from whatever value it's
// given *before* it declares the matching ROS parameters, so a YAML/CLI
// override on that declared parameter would only be reflected in `ros2
// param get`, not in the running controller -- overrideOrDefault() reads
// the node's raw parameter overrides ourselves so a config-file value
// actually reaches the initial gains, not just the parameter's reported value.
constexpr double kSteeringP = 1.5;
constexpr double kSteeringI = 0.05;
constexpr double kSteeringD = 0.15;
constexpr double kSteeringIClamp = 0.3;  // rad

constexpr double kSpeedP = 0.5;
constexpr double kSpeedI = 0.3;
constexpr double kSpeedD = 0.0;
constexpr double kSpeedIClamp = 0.5;  // m/s

double overrideOrDefault(rclcpp::Node & node, const std::string & name, double default_value)
{
  const auto & overrides = node.get_node_parameters_interface()->get_parameter_overrides();
  const auto it = overrides.find(name);
  return it != overrides.end() ? it->second.get<double>() : default_value;
}
}  // namespace

PidTracker::PidTracker(const rclcpp::NodeOptions & options)
: rclcpp::Node("pid_tracker", options),
  steering_pid_(
    get_node_base_interface(), get_node_logging_interface(), get_node_parameters_interface(),
    get_node_topics_interface(), "steering_pid", "steering_pid/", false),
  speed_pid_(
    get_node_base_interface(), get_node_logging_interface(), get_node_parameters_interface(),
    get_node_topics_interface(), "speed_pid", "speed_pid/", false)
{
  wheelbase_m_ = declare_parameter<double>("wheelbase_m", 0.65);
  max_steering_rad_ = declare_parameter<double>("max_steering_rad", 0.461);
  max_speed_mps_ = declare_parameter<double>("max_speed_mps", 1.0);
  control_frequency_hz_ = declare_parameter<double>("control_frequency_hz", 50.0);
  heading_feedforward_gain_ = declare_parameter<double>("heading_feedforward_gain", 1.0);
  enable_curvature_feedforward_ = declare_parameter<bool>("enable_curvature_feedforward", false);
  // Steering/braking react a beat late if they only ever see the *nearest*
  // waypoint: the steering actuator needs time to swing and the speed
  // profile's corner slowdown must start before the corner. Preview the
  // curvature feedforward and the target speed this far ahead along the path.
  curvature_preview_m_ = declare_parameter<double>("curvature_preview_m", 0.3);
  speed_preview_m_ = declare_parameter<double>("speed_preview_m", 2.5);
  controller_name_ = declare_parameter<std::string>("controller_name", "pid");

  control_toolbox::AntiWindupStrategy steering_aw;
  steering_aw.set_type("back_calculation");
  steering_aw.i_min = -kSteeringIClamp;
  steering_aw.i_max = kSteeringIClamp;
  steering_pid_.initialize_from_args(
    overrideOrDefault(*this, "steering_pid.p", kSteeringP),
    overrideOrDefault(*this, "steering_pid.i", kSteeringI),
    overrideOrDefault(*this, "steering_pid.d", kSteeringD),
    max_steering_rad_, -max_steering_rad_, steering_aw, false);

  control_toolbox::AntiWindupStrategy speed_aw;
  speed_aw.set_type("back_calculation");
  speed_aw.i_min = -kSpeedIClamp;
  speed_aw.i_max = kSpeedIClamp;
  speed_pid_.initialize_from_args(
    overrideOrDefault(*this, "speed_pid.p", kSpeedP),
    overrideOrDefault(*this, "speed_pid.i", kSpeedI),
    overrideOrDefault(*this, "speed_pid.d", kSpeedD),
    max_speed_mps_, -max_speed_mps_, speed_aw, false);

  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();
  trajectory_sub_ = create_subscription<hunter_msgs::msg::Trajectory>(
    "/trajectory/reference", latched_qos,
    std::bind(&PidTracker::trajectoryCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/ackermann_like_controller/odometry", rclcpp::QoS(10),
    std::bind(&PidTracker::odomCallback, this, std::placeholders::_1));

  cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
    "/ackermann_like_controller/reference", rclcpp::QoS(10));

  rclcpp::QoS diag_qos(20);
  diag_qos.reliable();
  diag_pub_ = create_publisher<hunter_msgs::msg::TrackingDiagnostics>("/tracking/diagnostics", diag_qos);

  // create_timer, NOT create_wall_timer: the loop must tick on the node's
  // clock so that under use_sim_time it runs at control_frequency_hz in
  // SIMULATED seconds. A wall timer ticks 50x per wall second regardless of
  // real-time factor; at RTF 0.3 that is ~170 ticks per simulated second with
  // a fixed dt of 1/50, which silently scales the effective integral gain up
  // and the derivative gain down by 1/RTF -- the expert's behaviour then
  // depends on host load. Measured on the Phase 6 gate lap before this fix:
  // 77,349 diagnostics in 455 s of sim time. Also what puts diag.header.stamp
  // on the same clock as /scan, which the dataset join requires.
  const auto period = std::chrono::duration<double>(1.0 / control_frequency_hz_);
  control_timer_ = create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PidTracker::controlLoop, this));

  RCLCPP_INFO(
    get_logger(), "pid_tracker ready: wheelbase=%.3f m, max_steering=%.3f rad, %.1f Hz",
    wheelbase_m_, max_steering_rad_, control_frequency_hz_);
}

void PidTracker::trajectoryCallback(const hunter_msgs::msg::Trajectory::SharedPtr msg)
{
  trajectory_ = *msg;
  has_trajectory_ = !trajectory_.waypoints.empty();
  first_tick_ = true;  // a new trajectory invalidates any previous nearest-index hint
  prev_nearest_index_ = 0;
}

void PidTracker::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_ = *msg;
  has_odom_ = true;
}

std::size_t PidTracker::previewIndex(std::size_t from_index, double distance_m) const
{
  const std::size_t wp_count = trajectory_.waypoints.size();
  std::size_t index = from_index;
  double travelled = 0.0;
  while (travelled < distance_m) {
    const std::size_t next = index + 1 < wp_count
      ? index + 1
      : (trajectory_.is_closed_loop ? 0 : index);
    if (next == index) {
      break;  // end of an open path
    }
    const auto & a = trajectory_.waypoints[index];
    const auto & b = trajectory_.waypoints[next];
    travelled += std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
    index = next;
  }
  return index;
}

void PidTracker::controlLoop()
{
  if (!has_trajectory_ || !has_odom_) {
    return;
  }

  const auto pose = hunter_control_common::poseFromOdometry(latest_odom_);
  const std::size_t search_window = first_tick_ ? 0 : 100;
  const auto err = hunter_control_common::computeTrackingError(
    trajectory_.waypoints, trajectory_.is_closed_loop, pose, prev_nearest_index_, search_window);
  prev_nearest_index_ = err.index;
  first_tick_ = false;

  const auto & target_wp = trajectory_.waypoints[err.index];

  // Steering feedforward and corner braking need *different* lookaheads, and
  // sharing one is actively harmful. Braking wants a long lead (the vehicle
  // needs metres to shed speed), but the curvature feedforward wants almost
  // none: it should describe the curve the front wheels are entering *now*.
  // Feeding it a far-ahead curvature makes the robot steer for a corner it
  // has not reached, which on a long sweeper drifts it steadily off-path --
  // measured as a slow build to >1.5 m of deviation and 16% steering
  // saturation on curvature so gentle (R = 20-100 m) the vehicle should not
  // have needed more than a few degrees of steering.
  const std::size_t steer_index = previewIndex(err.index, curvature_preview_m_);
  const std::size_t speed_index = previewIndex(err.index, speed_preview_m_);
  const auto & steer_preview_wp = trajectory_.waypoints[steer_index];

  // min(): slow down for an upcoming corner early, but never speed up early
  // past what the current path point allows.
  const double target_speed =
    std::min(target_wp.vx_mps, trajectory_.waypoints[speed_index].vx_mps);

  const auto dt = rclcpp::Duration::from_seconds(1.0 / control_frequency_hz_);

  // Steering: cross-track PID trim + heading feedforward (+ optional curvature
  // feedforward). See Phase 3 plan for why heading is feedforward, not a second PID.
  //
  // Sign: for a forward-moving bicycle model, yaw rate = v*tan(steering)/L, so
  // positive steering increases yaw. heading_error_rad = vehicle_yaw - path_yaw,
  // so a positive heading_error (vehicle rotated too far CCW of the path) needs
  // *negative* steering to reduce yaw back toward the path -- the feedforward
  // term must be the negative of the error, not the error itself.
  const double steer_pid_term = steering_pid_.compute_command(-err.lateral_error_m, dt);
  const double heading_ff = -heading_feedforward_gain_ * err.heading_error_rad;
  const double curvature_ff =
    enable_curvature_feedforward_ ? std::atan(wheelbase_m_ * steer_preview_wp.kappa_radpm) : 0.0;
  const double steering_cmd =
    std::clamp(steer_pid_term + heading_ff + curvature_ff, -max_steering_rad_, max_steering_rad_);

  // Speed: feedforward the target profile speed, PID trims residual error.
  const double speed_error = target_speed - pose.speed;
  const double speed_pid_term = speed_pid_.compute_command(speed_error, dt);
  const double speed_cmd = std::clamp(target_speed + speed_pid_term, 0.0, max_speed_mps_);

  geometry_msgs::msg::TwistStamped cmd;
  // Zero-stamped so AckermannSteeringController's reference_timeout is judged
  // against its own clock, not ROS<->Gazebo sim-time sync jitter.
  cmd.header.stamp.sec = 0;
  cmd.header.stamp.nanosec = 0;
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = speed_cmd;
  cmd.twist.angular.z = speed_cmd * std::tan(steering_cmd) / wheelbase_m_;
  cmd_pub_->publish(cmd);

  hunter_msgs::msg::TrackingDiagnostics diag;
  diag.header.stamp = now();
  diag.header.frame_id = trajectory_.header.frame_id;
  diag.controller_name = controller_name_;
  diag.lateral_error_m = err.lateral_error_m;
  diag.heading_error_rad = err.heading_error_rad;
  diag.target_speed_mps = target_speed;
  diag.actual_speed_mps = pose.speed;
  diag.speed_error_mps = speed_error;
  diag.commanded_steering_rad = steering_cmd;
  diag.commanded_speed_mps = speed_cmd;
  diag.path_s_m = err.path_s_m;
  diag.closest_waypoint_index = static_cast<uint32_t>(err.index);
  diag_pub_->publish(diag);
}

}  // namespace hunter_control
