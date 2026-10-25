#include "hunter_control/pid_shadow.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hunter_control::PidShadow>());
  rclcpp::shutdown();
  return 0;
}
