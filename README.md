# Robo Arm Workspace

This repository contains a modular ROS 2 (Jazzy) workspace for a robotic arm stack

## Packages

### robo_arm_description

Robot description and low-level hardware abstraction:

- URDF/Xacro robot model and meshes
- ros2_control xacro integration
- hardware plugin (`StepperSystemHardware`)
- RViz/Gazebo support config

#### Launch Files

- `rsp.launch.py`: Starts `robot_state_publisher` from xacro arguments (`HW_mode`, `cmd_mode`). Parameters: `HW_mode:=mock` (options: `mock|real|gazebo`), `cmd_mode:=position`.
- `rviz_sim.launch.py`: Starts RViz and joint-state-publisher GUI for manual model visualization/testing. Parameters: `HW_mode:=mock` (options: `mock|real|gazebo`), `cmd_mode:=position`.
- `sim.launch.py`: Starts either RViz or Foxglove bridge depending on `sim_mode`. Parameters: `sim_mode:=rviz` (options: `rviz|foxglove`).
- `gazebo_sim.launch.py`: Brings up Gazebo, spawns the robot from `robot_description`, and starts ros2_control with trajectory stack. Parameters: `HW_mode:=gazebo` (options: `mock|real|gazebo`), `cmd_mode:=position`, `use_sim_time:=true`.

### robo_arm_controller

Controller-specific runtime configuration:

- controller YAML files (trajectory/velocity/joy)
- controller manager launch workflows
- joystick-to-twist teleoperation launch

#### Launch Files

- `trajectory_controllers.launch.py`: Starts controller manager and spawns trajectory + joint state controllers. Parameters: `HW_mode:=mock` (options: `mock|real|gazebo`), `cmd_mode:=position`.
- `vel_controllers.launch.py`: Starts controller manager and spawns velocity-mode controller chain. Parameters: `HW_mode:=mock` (options: `mock|real|gazebo`), `cmd_mode:=position`.
- `twist_joy.launch.py`: Starts joystick and `teleop_twist_joy` remapped to `/cmd_vel_joy`. Parameters: `use_sim_time:=false`.

### robo_arm_moveit

Owns motion planning and manipulation behavior:

- MoveIt configs (SRDF, kinematics, joint limits, controllers)
- MoveIt launch entry points (`demo`, `move_group`, `moveit_rviz`, etc.)
- servo teleoperation launch
- C++ executables for keyboard servo and static pick-place task

#### Launch Files

- `demo.launch.py`: Standard MoveIt demo bringup (planning + visualization stack). Parameters: none.
- `move_group.launch.py`: Launches only the MoveIt `move_group` action server. Parameters: none.
- `moveit_rviz.launch.py`: Launches MoveIt RViz interface. Parameters: none.
- `rsp.launch.py`: MoveIt-configured robot state publisher bringup. Parameters: none.
- `setup_assistant.launch.py`: Opens MoveIt Setup Assistant. Parameters: none.
- `spawn_controllers.launch.py`: Spawns controllers expected by MoveIt config. Parameters: none.
- `static_virtual_joint_tfs.launch.py`: Publishes static virtual joint transforms from MoveIt config. Parameters: none.
- `warehouse_db.launch.py`: Starts MoveIt warehouse database server. Parameters: none.
- `teleop.launch.py`: Starts MoveIt Servo stack (`servo_node`, controllers, state publisher, RViz). Parameters: none (all settings read from package config files).
- `pick_n_place.launch.py`: Starts full task-planning environment for static pick-and-place demo (`move_group`, controllers, RViz, task executable). Parameters: none.

### robo_arm_vision

Owns perception and vision streaming:

- YOLO detection node (Python)
- RealSense launch integration
- depth-fused detections published as ROS topics and JSON stream

#### Launch Files

- `robo_arm_vision.launch.py`: Launches YOLO detection node and optionally RealSense driver; publishes annotated image, detections, and enriched JSON output. Parameters: `use_realsense:=true`, `size:=424x240`, `fs:=15`, `hef:=yolov8n.hef`, `stream_fps:=12`, `host:=0.0.0.0`, `port:=5000`, `confidence_threshold:=0.45`, `depth_unit_scale:=0.001`, `color_topic:=/camera/camera/color/image_raw`, `depth_topic:=/camera/camera/aligned_depth_to_color/image_raw`, `annotated_topic:=/camera/color/image_annotated`, `detections_topic:=/detections_2d`, `enriched_topic:=/detections_enriched`.
- `realsense_glove.launch.py`: Launches RealSense camera and Foxglove bridge for camera stream monitoring. Parameters: `size:=424x240`, `fs:=5`, `enable_depth:=false`, `pointcloud_enable:=false`, `align_depth_enable:=false`, `ip:=0.0.0.0`, `port:=8765`.

## TODO

1. adapt octomap to track camera imu
2. develop hardware accelerated yolo object detection
3. Integrate object detection with planning scene generation