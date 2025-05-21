#ifndef STEPPER_SYSTEM_HARDWARE_HPP_
#define STEPPER_SYSTEM_HARDWARE_HPP_

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

#include "robotic_Arm/visibility_control.h"
#include "robotic_arm/CAN_Comms.hpp"
#include "robotic_arm/Stepper.hpp"

namespace robotic_arm
{
    class StepperSystemHardware : public hardware_interface::SystemInterface
    {

    struct Config {
    
        std::string diff_motor_L_name = "";
        std::string diff_motor_R_name = "";

        float loop_rate = 0;
        std::string device = "";
        int CAN_rate = 0;
        int timeout_ms = 0;
    };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystemHardware)

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state);

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State & previous_state);

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

    private:
        Config cfg_;
        CANComms comms_;

        StepperMotor diff_motor_L_;
        StepperMotor diff_motor_R_;
    };
}

#endif  // STEPPER_SYSTEM_HARDWARE_HPP_