#include "hunter_control/pid_shadow.hpp"

#include <chrono>

namespace hunter_control
{

PidShadow::PidShadow(const rclcpp::NodeOptions & options)
: rclcpp::Node("pid_shadow", options), core_(*this)
{
  rclcpp::QoS latched_qos(1);
  latched_qos.reliable().transient_local();
  trajectory_sub_ = create_subscription<hunter_msgs::msg::Trajectory>(
    "/trajectory/reference", latched_qos,
    [this](hunter_msgs::msg::Trajectory::SharedPtr msg) {core_.setTrajectory(*msg);});

  // The teacher's privilege: remapped to /ground_truth/odometry by the launch,
  // exactly as pid_tracker is when it drives.
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/ackermann_like_controller/odometry", rclcpp::QoS(10),
    [this](nav_msgs::msg::Odometry::SharedPtr msg) {core_.setOdometry(*msg);});

  rclcpp::QoS qos(20);
  qos.reliable();
  relabel_pub_ = create_publisher<hunter_msgs::msg::TrackingDiagnostics>("/expert/relabel", qos);

  const auto period = std::chrono::duration<double>(1.0 / core_.controlFrequencyHz());
  timer_ = create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&PidShadow::tick, this));

  RCLCPP_INFO(
    get_logger(), "pid_shadow ready: relabelling on /expert/relabel at %.1f Hz; "
    "this node has no command publisher", core_.controlFrequencyHz());
}

void PidShadow::tick()
{
  // The integrator runs along the STUDENT's trajectory: a sustained student
  // offset winds it up (clamped at 0.3 rad), so the relabel is "what the
  // expert would do having watched this drift", not a memoryless snapshot.
  // That is the expert's genuine response, and it points back to the line.
  const auto step = core_.step();
  if (step) {
    relabel_pub_->publish(core_.diagnostics(*step));
  }
}

}  // namespace hunter_control
