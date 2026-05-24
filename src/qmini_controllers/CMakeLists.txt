cmake_minimum_required(VERSION 3.16)
project(qmini_controllers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(qmini_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(pd_packer_node src/pd_packer_node.cpp)
ament_target_dependencies(pd_packer_node rclcpp sensor_msgs qmini_msgs)
target_link_libraries(pd_packer_node yaml-cpp)
install(TARGETS pd_packer_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
