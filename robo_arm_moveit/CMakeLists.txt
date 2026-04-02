cmake_minimum_required(VERSION 3.22)
project(robo_arm_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# Servo keyboard input node
add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies( servo_keyboard_input
  rclcpp
  geometry_msgs
  control_msgs
  moveit_servo
)

install(
  TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)

# MTC node
add_executable(static_pick_place src/static_pick_place.cpp)
target_include_directories(static_pick_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(static_pick_place PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies( static_pick_place
  "moveit_task_constructor_core"
  "rclcpp"
)

install(
  TARGETS static_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

# Detection scene builder node
add_executable(detection_scene_builder src/detection_scene_builder.cpp)
target_include_directories(detection_scene_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(detection_scene_builder PUBLIC cxx_std_17)
ament_target_dependencies(detection_scene_builder
  rclcpp
  vision_msgs
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
)

install(
  TARGETS detection_scene_builder
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()