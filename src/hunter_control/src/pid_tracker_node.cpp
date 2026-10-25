#include "hunter_control/pid_tracker.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hunter_control::PidTracker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
