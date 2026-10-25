#ifndef HUNTER_CONTROL__PID_CORE_HPP_
#define HUNTER_CONTROL__PID_CORE_HPP_

#include <optional>
#include <string>

#include "control_toolbox/pid_ros.hpp"
#include "hunter_control/hunter_control_common.hpp"
#include "hunter_msgs/msg/tracking_diagnostics.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_control
{

// The PID expert's control law and nothing else.
//
// PidCore declares the parameters, owns the live-tunable gains, keeps the
// nearest-index hint and the PID integrator state, and turns (trajectory,
// odometry) into (steering, speed). It has NO publishers and no timer: the
// node that owns it decides what happens to the numbers. That is the whole
// point of the split -- pid_tracker publishes them as a command,
// pid_shadow publishes them as a relabel, and the shadow cannot publish a
// command because there is nothing in it that could.
class PidCore
{
public:
  struct Step
  {
    hunter_control_common::TrackingErrorResult err;
    double target_speed_mps = 0.0;
    double actual_speed_mps = 0.0;
    double speed_error_mps = 0.0;
    double steering_cmd_rad = 0.0;  // post-clamp
    double speed_cmd_mps = 0.0;     // post-clamp
  };

  explicit PidCore(rclcpp::Node & node);

  void setTrajectory(const hunter_msgs::msg::Trajectory & msg);
  void setOdometry(const nav_msgs::msg::Odometry & msg);
  bool ready() const {return has_trajectory_ && has_odom_;}

  // One control tick at control_frequency_hz. nullopt until both inputs exist.
  std::optional<Step> step();

  // Fills every field of a TrackingDiagnostics from a Step, stamped with the
  // node clock, so both owners publish byte-identical diagnostics.
  hunter_msgs::msg::TrackingDiagnostics diagnostics(const Step & s) const;

  double wheelbase() const {return wheelbase_m_;}
  double controlFrequencyHz() const {return control_frequency_hz_;}
  const std::string & controllerName() const {return controller_name_;}

private:
  std::size_t previewIndex(std::size_t from_index, double distance_m) const;

  rclcpp::Node & node_;
  double wheelbase_m_;
  double max_steering_rad_;
  double max_speed_mps_;
  double control_frequency_hz_;
  double heading_feedforward_gain_;
  bool enable_curvature_feedforward_;
  double curvature_preview_m_;
  double speed_preview_m_;
  std::string controller_name_;

  control_toolbox::PidROS steering_pid_;
  control_toolbox::PidROS speed_pid_;

  hunter_msgs::msg::Trajectory trajectory_;
  bool has_trajectory_ = false;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_ = false;
  std::size_t prev_nearest_index_ = 0;
  bool first_tick_ = true;
};

}  // namespace hunter_control

#endif  // HUNTER_CONTROL__PID_CORE_HPP_
