#include "dummy_controller/dummy_controller.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyController>());
  rclcpp::shutdown();
  return 0;
}