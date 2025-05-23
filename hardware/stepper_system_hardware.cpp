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
        const hardware_interface::HardwareInfo & info) {
        
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(logger_, "info Failed!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.diff_motor_L_name = info_.hardware_parameters["diff_motor_L_name"];
        cfg_.diff_motor_R_name = info_.hardware_parameters["diff_motor_R_name"];
        cfg_.device = info_.hardware_parameters["device"];
        cfg_.CAN_rate = std::stoi(info_.hardware_parameters["CAN_rate"]);

        diff_motor_L_.name = cfg_.diff_motor_L_name;
        diff_motor_R_.name = cfg_.diff_motor_R_name;

        RCLCPP_INFO(logger_, "info passed successfully!");

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
          const auto & joint_name = joint.name;
          const auto & cmds = joint.command_interfaces;
          const auto & states = joint.state_interfaces;
              
          // Allow 2 or 3 command interfaces
          if (cmds.size() != 2 && cmds.size() != 3)
          {
              RCLCPP_FATAL(
                  logger_,"Joint '%s' has %zu command interfaces. Expected 2 (speed mode) or 3 (position mode).",
                  joint_name.c_str(), cmds.size());
              return hardware_interface::CallbackReturn::ERROR;
          }
        
          if (cmds.size() == 2 &&
              cmds[0].name == hardware_interface::HW_IF_VELOCITY &&
              cmds[1].name == hardware_interface::HW_IF_ACCELERATION)
          {
              cmd_mode_ = "speed";
          }
          else if (cmds.size() == 3 &&
                   cmds[0].name == hardware_interface::HW_IF_POSITION &&
                   cmds[1].name == hardware_interface::HW_IF_VELOCITY &&
                   cmds[2].name == hardware_interface::HW_IF_ACCELERATION)
          {
              cmd_mode_ = "position";
          }
          else
          {
              RCLCPP_FATAL(
                  logger_, "Joint '%s' has unsupported command interface configuration.", joint_name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
        
          // Expect 2 state interfaces: position and velocity
          if (states.size() != 2 ||
              states[0].name != hardware_interface::HW_IF_POSITION ||
              states[1].name != hardware_interface::HW_IF_VELOCITY)
          {
              RCLCPP_FATAL(
                  logger_, "Joint '%s' must have exactly 2 state interfaces: 'position' and 'velocity'.", joint_name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
        }      

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // hardware_interface::CallbackReturn StepperSystemHardware::on_configure(
    //         const rclcpp_lifecycle::State & previous_state) 
    // {
    // }

    // hardware_interface::CallbackReturn StepperSystemHardware::on_cleanup(
    //         const rclcpp_lifecycle::State & previous_state) 
    // {
    
    // }
    
    std::vector<hardware_interface::StateInterface> StepperSystemHardware::export_state_interfaces() 
    {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          diff_motor_L_.name, hardware_interface::HW_IF_POSITION, &diff_motor_L_.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          diff_motor_L_.name, hardware_interface::HW_IF_VELOCITY, &diff_motor_L_.vel));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          diff_motor_R_.name, hardware_interface::HW_IF_POSITION, &diff_motor_R_.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        diff_motor_R_.name, hardware_interface::HW_IF_VELOCITY, &diff_motor_R_.vel));

      return state_interfaces;
    }

  std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // If in position mode, export position interfaces first
    if (cmd_mode_ == "position")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          diff_motor_L_.name, hardware_interface::HW_IF_POSITION, &diff_motor_L_.cmd_pos));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          diff_motor_R_.name, hardware_interface::HW_IF_POSITION, &diff_motor_R_.cmd_pos));
    }

    // Always export velocity and acceleration
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        diff_motor_L_.name, hardware_interface::HW_IF_VELOCITY, &diff_motor_L_.cmd_vel));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        diff_motor_R_.name, hardware_interface::HW_IF_VELOCITY, &diff_motor_R_.cmd_vel));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        diff_motor_L_.name, hardware_interface::HW_IF_ACCELERATION, &diff_motor_L_.cmd_acc));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        diff_motor_R_.name, hardware_interface::HW_IF_ACCELERATION, &diff_motor_R_.cmd_acc));

    return command_interfaces;
  }


    hardware_interface::CallbackReturn StepperSystemHardware::on_activate(
            const rclcpp_lifecycle::State & /*previous_state*/) 
    {
      RCLCPP_INFO(logger_, "Activating ...please wait...");

      // connect to arduino with device, baud_rate and timeout_ms parameters
      comms_.connect(cfg_.device, cfg_.CAN_rate);

      RCLCPP_INFO(logger_, "Successfully activated!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_deactivate(
            const rclcpp_lifecycle::State & /*previous_state*/) 
    {
      RCLCPP_INFO(logger_, "Deactivating hardware interface...");

      // connect to arduino with device, baud_rate and timeout_ms parameters
      comms_.disconnect(cfg_.device);

      RCLCPP_INFO(logger_, "Successfully deactivated!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type StepperSystemHardware::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {
        // Read the current speed of the motors
        diff_motor_L_.vel = diff_motor_L_.read_speed();
        diff_motor_R_.vel = diff_motor_R_.read_speed();

        // Read the current position of the motors
        diff_motor_L_.pos = diff_motor_L_.read_encoder_position();
        diff_motor_R_.pos = diff_motor_R_.read_encoder_position();

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type StepperSystemHardware::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {
      if (cmd_mode_ == "speed")
      {
          bool dir = diff_motor_L_.cmd_vel >= 0;
          uint16_t rpm = static_cast<uint16_t>(std::abs(diff_motor_L_.cmd_vel));
          uint8_t acc = static_cast<uint8_t>(diff_motor_L_.cmd_acc);

          diff_motor_L_.send_speed(rpm, acc, dir);

          dir = diff_motor_R_.cmd_vel >= 0;
          rpm = static_cast<uint16_t>(std::abs(diff_motor_R_.cmd_vel));
          acc = static_cast<uint8_t>(diff_motor_R_.cmd_acc);

          diff_motor_R_.send_speed(rpm, acc, dir);
      }
      else {

          int32_t abs_pos = static_cast<int32_t>(diff_motor_L_.cmd_pos);
          uint16_t rpm = static_cast<uint16_t>(std::abs(diff_motor_L_.cmd_vel));  
          uint8_t acc = static_cast<uint8_t>(diff_motor_L_.cmd_acc);

          diff_motor_L_.send_absolute_position(abs_pos, rpm, acc);

          abs_pos = static_cast<int32_t>(diff_motor_R_.cmd_pos);
          rpm = static_cast<uint16_t>(std::abs(diff_motor_R_.cmd_vel));
          acc = static_cast<uint8_t>(diff_motor_R_.cmd_acc);

          diff_motor_R_.send_absolute_position(abs_pos, rpm, acc);
      }

      return hardware_interface::return_type::OK;
    }

} // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotic_arm::StepperSystemHardware, hardware_interface::SystemInterface)