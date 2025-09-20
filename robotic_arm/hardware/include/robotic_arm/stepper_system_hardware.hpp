#ifndef ROBOTIC_ARM_STEPPER_SYSTEM_HARDWARE_HPP_
#define ROBOTIC_ARM_STEPPER_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "robotic_arm/visibility_control.h"
#include "robotic_arm/CAN_Comms.hpp"
#include "robotic_arm/Stepper.hpp"

namespace robotic_arm
{   

    class StepperSystemHardware : public hardware_interface::SystemInterface
    {

        struct Config {
        
            std::vector<std::string> joint_names;
            std::vector<double> joint_reductions;
            std::vector<uint16_t>CAN_id;

            std::string interface_name = "";
            int CAN_rate = 0;
            int timout_ms = 0;
            int loop_rate = 0;
        };

        enum CommandMode { SPEED, POSITION };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystemHardware)

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state);

        // ROBOTIC_ARM_PUBLIC
        // hardware_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State & previous_state);

        ROBOTIC_ARM_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        ROBOTIC_ARM_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        // ROBOTIC_ARM_PUBLIC
        // hardware_interface::return_type prepare_command_mode_switch(
        //     const std::vector<std::string>& start_ifaces, 
        //     const std::vector<std::string>& stop_ifaces) override;

        // ROBOTIC_ARM_PUBLIC
        // hardware_interface::return_type perform_command_mode_switch(
        //     const std::vector<std::string>& start_ifaces, 
        //     const std::vector<std::string>& stop_ifaces) override;

    private:
        
        void setCommandMode(CommandMode mode) { 
            cmd_mode_ = mode;
        }

        void updateDiffJointStates();
        double updateDiffMotorCmd(size_t motor_index, size_t cmd_index);

        Config cfg_;
        rclcpp::Logger logger_ = rclcpp::get_logger("stepper_system_hardware");  
        CANComms comms_{logger_}; 

        std::vector<std::unique_ptr<StepperMotor>> motors;
        std::vector<std::array<double, 2>> joint_states;   // position, velocity
        std::vector<std::array<double, 3>> joint_cmds;     // position, velocity, acceleration
        CommandMode cmd_mode_;
        CommandMode requested_cmd_mode_;
    };
}

#endif  // ROBOTIC_ARM_STEPPER_SYSTEM_HARDWARE_HPP_