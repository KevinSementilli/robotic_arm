#include "robotic_arm/stepper_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotic_arm {

    hardware_interface::CallbackReturn StepperSystemHardware::on_init(
        const hardware_interface::HardwareInfo & info) 
    {
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("ServoSystemHardware"), "info Failed!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.diff_motor_L_name = info.hardware_parameters["diff_motor_L_name"];
        cfg_.diff_motor_R_name = info.hardware_parameters["diff_motor_R_name"];

        cfg_.loop_rate = std::stof(info.hardware_parameters["loop_rate"]);
        cfg_.device = info.hardware_parameters["device"];
        cfg_.CAN_rate = std::stoi(info.hardware_parameters["CAN_rate"]);
        cfg_.timeout_ms = std::stoi(info.hardware_parameters["timeout_ms"]);

        RCLCPP_INFO(rclcpp::get_logger("ServoSystemHardware"), "info passed successfully!");

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
        
            if (joint.command_interfaces.size() != 1)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 2)
            {
              RCLCPP_FATAL(
                rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
              return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
              RCLCPP_FATAL(
                rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
              return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
            {
              RCLCPP_FATAL(
                rclcpp::get_logger("ServoSystemHardware"), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
              return hardware_interface::CallbackReturn::ERROR;
            }
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_configure(
            const rclcpp_lifecycle::State & previous_state) 
    {

    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_cleanup(
            const rclcpp_lifecycle::State & previous_state) 
    {
    
    }
    
    std::vector<hardware_interface::StateInterface> StepperSystemHardware::export_state_interfaces() 
    {

    }

    std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces() 
    {
    
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_activate(
            const rclcpp_lifecycle::State & previous_state) 
    {
    
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_deactivate(
            const rclcpp_lifecycle::State & previous_state) 
    {

    }

    hardware_interface::return_type StepperSystemHardware::read(
            const rclcpp::Time & time, const rclcpp::Duration & period) 
    {
        std::stringstream feedback;
        feedback << 

    }

    hardware_interface::return_type StepperSystemHardware::write(
            const rclcpp::Time & time, const rclcpp::Duration & period) 
    {

    }

} // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotic_arm::StepperSystemHardware, hardware_interface::SystemInterface)