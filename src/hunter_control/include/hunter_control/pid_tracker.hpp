#ifndef HUNTER_CONTROL__PID_TRACKER_HPP_
#define HUNTER_CONTROL__PID_TRACKER_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hunter_control/pid_core.hpp"
#include "hunter_msgs/msg/tracking_diagnostics.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_control
{

// The expert as a controller: PidCore's output goes to the robot.
class PidTracker : public rclcpp::Node
{
public:
  explicit PidTracker(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void controlLoop();

  PidCore core_;
  rclcpp::Subscription<hunter_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<hunter_msgs::msg::TrackingDiagnostics>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace hunter_control

#endif  // HUNTER_CONTROL__PID_TRACKER_HPP_
