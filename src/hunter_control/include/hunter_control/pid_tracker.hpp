#ifndef HUNTER_CONTROL__PID_TRACKER_HPP_
#define HUNTER_CONTROL__PID_TRACKER_HPP_

#include "control_toolbox/pid_ros.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "hunter_msgs/msg/tracking_diagnostics.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_control
{

class PidTracker : public rclcpp::Node
{
public:
  explicit PidTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void trajectoryCallback(const hunter_msgs::msg::Trajectory::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void controlLoop();

  // Index of the waypoint distance_m further along the path from from_index,
  // wrapping on a closed loop and clamping at the end of an open one.
  std::size_t previewIndex(std::size_t from_index, double distance_m) const;

  // Params
  double wheelbase_m_;
  double max_steering_rad_;
  double max_speed_mps_;
  double control_frequency_hz_;
  double heading_feedforward_gain_;
  bool enable_curvature_feedforward_;
  double curvature_preview_m_;  // steering feedforward lookahead (short)
  double speed_preview_m_;      // corner-braking lookahead (long)
  std::string controller_name_;

  control_toolbox::PidROS steering_pid_;
  control_toolbox::PidROS speed_pid_;

  rclcpp::Subscription<hunter_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<hunter_msgs::msg::TrackingDiagnostics>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  hunter_msgs::msg::Trajectory trajectory_;
  bool has_trajectory_ = false;
  nav_msgs::msg::Odometry latest_odom_;
  bool has_odom_ = false;
  std::size_t prev_nearest_index_ = 0;
  bool first_tick_ = true;
};

}  // namespace hunter_control

#endif  // HUNTER_CONTROL__PID_TRACKER_HPP_
