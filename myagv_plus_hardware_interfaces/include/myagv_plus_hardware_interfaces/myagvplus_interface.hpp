#ifndef MYAGVPLUS_INTERFACE_H
#define MYAGVPLUS_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include "rclcpp/macros.hpp"
#include <hardware_interface/system_interface.hpp>

namespace myagvplus_hardware_interfaces
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MyAGVPlusInterface : public hardware_interface::SystemInterface
{
public:
    MyAGVPlusInterface();
    virtual ~MyAGVPlusInterface();
    
    // LifecycleNodeInterface
    CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    
    // SystemInterface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
};

} // namespace myagvplus_hardware_interfaces

#endif // MYAGVPLUS_INTERFACE_H