#include "hunter_control/pid_tracker.hpp"

#include <chrono>
#include <cmath>

namespace hunter_control
{

PidTracker::PidTracker(const rclcpp::NodeOptions & options)
: rclcpp::Node("pid_tracker", options), core_(*this)
{
  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();
  trajectory_sub_ = create_subscription<hunter_msgs::msg::Trajectory>(
    "/trajectory/reference", latched_qos,
    [this](hunter_msgs::msg::Trajectory::SharedPtr msg) {core_.setTrajectory(*msg);});

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/ackermann_like_controller/odometry", rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {core_.setOdometry(*msg);});

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
  const auto period = std::chrono::duration<double>(1.0 / core_.controlFrequencyHz());
  control_timer_ = create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PidTracker::controlLoop, this));

  RCLCPP_INFO(
    get_logger(), "pid_tracker ready: wheelbase=%.3f m, %.1f Hz",
    core_.wheelbase(), core_.controlFrequencyHz());
}

void PidTracker::controlLoop()
{
  const auto step = core_.step();
  if (!step) {
    return;
  }
  geometry_msgs::msg::TwistStamped cmd;
  // Zero-stamped so AckermannSteeringController's reference_timeout is judged
  // against its own clock, not ROS<->Gazebo sim-time sync jitter.
  cmd.header.stamp.sec = 0;
  cmd.header.stamp.nanosec = 0;
  cmd.header.frame_id = "base_link";
  cmd.twist.linear.x = step->speed_cmd_mps;
  cmd.twist.angular.z = step->speed_cmd_mps * std::tan(step->steering_cmd_rad) / core_.wheelbase();
  cmd_pub_->publish(cmd);
  diag_pub_->publish(core_.diagnostics(*step));
}

}  // namespace hunter_control
