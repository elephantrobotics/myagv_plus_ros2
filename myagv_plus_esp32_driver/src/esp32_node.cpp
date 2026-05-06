#include "myagv_plus_esp32_driver/esp32_driver.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyAGV_Plus>("myagv_plus_esp32_node");

  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}