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

        cfg_.joint_names.reserve(info_.joints.size());
        cfg_.joint_reductions.reserve(info_.joints.size());

        // Extract joint names and reductions from hardware parameters
        for (const auto &joint : info_.joints) {
            cfg_.joint_names.push_back(joint.name);
        
            // reduction is expected to be provided as "<joint_name>_reduction" in hardware parameters
            auto reduction_key = joint.name + "_reduction";
            if (info_.hardware_parameters.count(reduction_key)) {
                cfg_.joint_reductions.push_back(std::stod(info_.hardware_parameters.at(reduction_key)));
            } else {
                RCLCPP_WARN(logger_, "No reduction specified for %s, defaulting to 1.0", joint.name.c_str());
                cfg_.joint_reductions.push_back(1.0f);
            }
        }

        cfg_.interface_name = info_.hardware_parameters["interface_name"];
        cfg_.CAN_rate = std::stoi(info_.hardware_parameters["CAN_rate"]);

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
              RCLCPP_INFO(logger_, "Hardware interface is in speed mode");
          }
          else if (cmds.size() == 3 &&
                   cmds[0].name == hardware_interface::HW_IF_POSITION &&
                   cmds[1].name == hardware_interface::HW_IF_VELOCITY &&
                   cmds[2].name == hardware_interface::HW_IF_ACCELERATION)
          {
              cmd_mode_ = "position";
              RCLCPP_INFO(logger_, "Hardware interface is in position mode");
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

    hardware_interface::CallbackReturn StepperSystemHardware::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {
        
        motors.clear();
        motors.reserve(cfg_.joint_names.size());

        RCLCPP_INFO(logger_, "Configuring %zu stepper motors...",
                    cfg_.joint_names.size());

        for (size_t i = 0; i < cfg_.joint_names.size(); i++) {
            
            uint16_t CAN_id = 0x00 + i;
            RCLCPP_INFO(logger_, "[%zu] Joint '%s' assigned CAN ID: 0x%X", i, cfg_.joint_names[i].c_str(), CAN_id);
            motors.emplace_back(cfg_.joint_names[i], comms_, logger_, CAN_id);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // hardware_interface::CallbackReturn StepperSystemHardware::on_cleanup(
    //         const rclcpp_lifecycle::State & previous_state) 
    // {
    
    // }

    hardware_interface::CallbackReturn StepperSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        
        RCLCPP_INFO(logger_, "Activating CAN device: %s, bitrate: %d", cfg_.interface_name.c_str(), cfg_.CAN_rate);

        while(!comms_.connect(cfg_.interface_name, cfg_.CAN_rate)) {

            RCLCPP_WARN(logger_, "CAN device not connected, retrying ...");
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_INFO(logger_, "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {

      RCLCPP_INFO(logger_, "Deactivating hardware interface...");
      comms_.disconnect(cfg_.interface_name);
      RCLCPP_INFO(logger_, "Successfully deactivated!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> StepperSystemHardware::export_state_interfaces() {
        
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (int i = 0; i < motors.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_POSITION, &states[i][0]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_VELOCITY, &states[i][1]));
        }

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces() {
        
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // If in position mode, export position interfaces first
        if (cmd_mode_ == "position")
        {
            for (int i = 0; i < motors.size(); i++) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    cfg_.joint_names[i], hardware_interface::HW_IF_POSITION, &commands[i][0]));
            }
        }
        
        for (int i = 0; i < motors.size(); i++) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_VELOCITY, &commands[i][1]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_ACCELERATION, &commands[i][2]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type StepperSystemHardware::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {   
        for (int i = 0; i < motors.size(); i++) {
            motors[i].pos = motors[i].read_encoder_position();
            motors[i].vel = motors[i].read_speed();
        }

        for (int i = 0; i < motors.size() - 2; i++) {
            states[i][0] = motors[i].pos / cfg_.joint_reductions[i];
            states[i][1] = motors[i].vel / cfg_.joint_reductions[i];
        }

        // differential gearbox mechanism
        states[4][0] = (motors[4].pos / cfg_.joint_reductions[4] + motors[5].pos / cfg_.joint_reductions[4]) / 2;
        states[4][1] = (motors[4].vel / cfg_.joint_reductions[4] + motors[5].vel / cfg_.joint_reductions[4]) / 2;

        states[5][0] = (motors[4].pos / cfg_.joint_reductions[4] - motors[5].pos / cfg_.joint_reductions[4]) / 2;
        states[5][1] = (motors[4].vel / cfg_.joint_reductions[4] - motors[5].vel / cfg_.joint_reductions[4]) / 2;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type StepperSystemHardware::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        
        if (cmd_mode_ == "position") {
            
            for (int i = 0; i < motors.size() - 2; i++) {
                if(!motors[i].send_absolute_position(
                    static_cast<int32_t>(commands[i][0] * cfg_.joint_reductions[i]),
                    static_cast<uint16_t>(commands[i][1] * cfg_.joint_reductions[i]),
                    static_cast<uint8_t>(commands[i][2] * cfg_.joint_reductions[i])))
                {
                    RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motors[i].name_.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }

            differentialPositionCommand();

        } else {
            
            for(int i = 0; i < motors.size() - 2; i++) {
                if(!motors[i].send_speed(
                    static_cast<uint16_t>(commands[i][1] * cfg_.joint_reductions[i]),
                    static_cast<uint8_t>(commands[i][2] * cfg_.joint_reductions[i]),
                    commands[i][1] > 0))
                {
                    RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motors[i].name_.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }

            differentialSpeedCommand();
        }

        return hardware_interface::return_type::OK;
    }
    
    hardware_interface::return_type StepperSystemHardware::differentialPositionCommand() {

        double carrier_pos = commands[4][0];
        double carrier_vel = commands[4][1];
        double carrier_acc = commands[4][2];
            
        double bevel_pos = commands[5][0];
        double bevel_vel = commands[5][1];
        double bevel_acc = commands[5][2];

        if(!motors[4].send_absolute_position(
            static_cast<int32_t>(cfg_.joint_reductions[4] * (carrier_pos + bevel_pos)),
            static_cast<uint16_t>(cfg_.joint_reductions[4] * (carrier_vel + bevel_vel)),
            static_cast<uint8_t>(cfg_.joint_reductions[4] * (carrier_acc + bevel_acc))))
        {
            RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motors[4].name_.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if(!motors[5].send_absolute_position(
            static_cast<int32_t>(cfg_.joint_reductions[5] * (carrier_pos - bevel_pos)),
            static_cast<uint16_t>(cfg_.joint_reductions[5] * (carrier_vel - bevel_vel)),
            static_cast<uint8_t>(cfg_.joint_reductions[5] * (carrier_acc - bevel_acc))))
        {
            RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motors[4].name_.c_str());
            return hardware_interface::return_type::ERROR;
        }
    } 

    hardware_interface::return_type StepperSystemHardware::differentialSpeedCommand() {

        double carrier_vel = commands[4][1];
        double carrier_acc = commands[4][2];
            
        double bevel_vel = commands[5][1];
        double bevel_acc = commands[5][2];

        if(!motors[4].send_speed(
            static_cast<uint16_t>(cfg_.joint_reductions[4] * (carrier_vel + bevel_vel)),
            static_cast<uint8_t>(cfg_.joint_reductions[4] * (carrier_acc + bevel_acc))))
        {
            RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motors[4].name_.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if(!motors[5].send_speed(
            static_cast<uint16_t>(cfg_.joint_reductions[5] * (carrier_vel - bevel_vel)),
            static_cast<uint8_t>(cfg_.joint_reductions[5] * (carrier_acc - bevel_acc))))
        {
            RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motors[5].name_.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }

} // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robotic_arm::StepperSystemHardware, hardware_interface::SystemInterface)