#ifndef MYAGVPLUS_INTERFACE_H
#define MYAGVPLUS_INTERFACE_H

#include <memory>
#include <string>
#include <vector>

#include "myagv_plus_hardware_interfaces/damiao.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp/macros.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

namespace myagvplus_hardware_interfaces
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

struct MotorDesc
{
  std::string joint_name;
  uint32_t can_id;   // Slave ID
  uint32_t mst_id;   // Master ID
  std::unique_ptr<damiao::Motor> motor;
};

class MyAGVPlusInterface : public hardware_interface::SystemInterface
{
public:
    // LifecycleNodeInterface
    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
    
    // SystemInterface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:

  std::string port_;
  int baudrate_{0};
  std::vector<MotorDesc> motors_;

  std::shared_ptr<SerialPort> serial_;
  std::shared_ptr<damiao::Motor_Control> motor_ctrl_;

  // Position state storage for all joints
  std::vector<double> position_states_;

  // Velocity command and state storage for wheels
  std::vector<double> velocity_commands_;
  std::vector<double> velocity_states_;

};

} // namespace myagvplus_hardware_interfaces

#endif // MYAGVPLUS_INTERFACE_H