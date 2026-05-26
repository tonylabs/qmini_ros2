cmake_minimum_required(VERSION 3.16)
project(qmini_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- imu_driver_node ---
# Self-contained: termios serial + FDILink frame parser. No external serial lib.
add_executable(imu_driver_node
  src/imu_driver_node.cpp
  src/fdilink_crc.cpp
)
target_include_directories(imu_driver_node PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
ament_target_dependencies(imu_driver_node rclcpp sensor_msgs)
install(TARGETS imu_driver_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
