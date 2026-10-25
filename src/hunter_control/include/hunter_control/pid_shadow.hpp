#ifndef HUNTER_CONTROL__PID_SHADOW_HPP_
#define HUNTER_CONTROL__PID_SHADOW_HPP_

#include "hunter_control/pid_core.hpp"
#include "hunter_msgs/msg/tracking_diagnostics.hpp"
#include "hunter_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hunter_control
{

// The expert as a labeller (DAgger). Same PidCore, same parameters, same
// ground-truth pose -- but the only publisher this class has is the relabel.
// There is no TwistStamped publisher member, no include of the message, and
// nothing to call publish() on: a shadow that steers the robot is not a
// configuration this type can express. That is deliberate. The README's
// first troubleshooting entry is two trackers commanding the robot at once;
// DAgger runs an expert next to the student by design, so "the shadow cannot
// command" has to be a property of the type, not of a flag.
class PidShadow : public rclcpp::Node
{
public:
  explicit PidShadow(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();

  PidCore core_;
  rclcpp::Subscription<hunter_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<hunter_msgs::msg::TrackingDiagnostics>::SharedPtr relabel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace hunter_control

#endif  // HUNTER_CONTROL__PID_SHADOW_HPP_
