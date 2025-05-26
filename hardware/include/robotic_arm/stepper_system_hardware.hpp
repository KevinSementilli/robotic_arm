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
    
        std::string carrier_name = "";
        std::string central_bevel_name = "";

        std::string device = "";
        int CAN_rate = 0;
        int timout_ms = 0;
    };

    struct Carrier {
        std::string name = "";
        double cmd_pos = 0;
        double cmd_vel = 0;
        double cmd_acc = 0;
        double pos = 0;
        double vel = 0;
        double acc = 0;
    };

    struct CentralBevel {
        std::string name = "";
        double cmd_pos = 0;
        double cmd_vel = 0;
        double cmd_acc = 0;
        double pos = 0;
        double vel = 0;
        double acc = 0;
    };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystemHardware)

        ROBOTIC_ARM_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        // ROBOTIC_ARM_PUBLIC
        // hardware_interface::CallbackReturn on_configure(
        //     const rclcpp_lifecycle::State & previous_state);

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

        ROBOTIC_ARM_PUBLIC
        void set_motor_cmd()
        {   
            diff_motor_L_.cmd_pos = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_pos + central_bevel_.cmd_pos);
            diff_motor_L_.cmd_vel = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_vel + central_bevel_.cmd_vel);
            diff_motor_L_.cmd_acc = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_acc + central_bevel_.cmd_acc);    

            diff_motor_R_.cmd_pos = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_pos - central_bevel_.cmd_pos);
            diff_motor_R_.cmd_vel = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_vel - central_bevel_.cmd_vel);
            diff_motor_R_.cmd_acc = (pulley_ratio_ * bevel_gear_ratio_)*(carrier_.cmd_acc - central_bevel_.cmd_acc);
        }

    private:
        Config cfg_;
        rclcpp::Logger logger_ = rclcpp::get_logger("stepper_system_hardware");  
        CANComms comms_{logger_}; 

        StepperMotor diff_motor_L_{0x01, comms_, logger_};
        StepperMotor diff_motor_R_{0x02, comms_, logger_};

        Carrier carrier_;
        CentralBevel central_bevel_;
        const double pulley_ratio_ = 1.8;
        const double bevel_gear_ratio_ = 2.0;

        std::string cmd_mode_ = "";
    };
}

#endif  // ROBOTIC_ARM_STEPPER_SYSTEM_HARDWARE_HPP_