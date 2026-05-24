cmake_minimum_required(VERSION 3.8)
project(qmini_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(msg_files
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/JoystickCommand.msg"
  "msg/SafetyHeartbeat.msg"
  "msg/MotionGate.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
