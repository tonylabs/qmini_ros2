cmake_minimum_required(VERSION 3.16)
project(qmini_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(qmini_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- Unitree SDK (vendored as a git submodule, see third_party/README.md) ---
set(UNITREE_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/unitree_actuator_sdk")
if(NOT EXISTS "${UNITREE_SDK_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "unitree_actuator_sdk not found at ${UNITREE_SDK_DIR}.\n"
    "Run from the workspace root:\n"
    "  git submodule add https://github.com/unitreerobotics/unitree_actuator_sdk \\\n"
    "    src/qmini_hardware/third_party/unitree_actuator_sdk\n"
    "Then re-run colcon build."
  )
endif()

# The SDK ships prebuilt shared libraries — we link against the right one for
# the host architecture rather than building from source (we don't have it).
if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(UNITREE_SDK_LIB "${UNITREE_SDK_DIR}/lib/libUnitreeMotorSDK_Arm64.so")
else()
  set(UNITREE_SDK_LIB "${UNITREE_SDK_DIR}/lib/libUnitreeMotorSDK_Linux64.so")
endif()
if(NOT EXISTS "${UNITREE_SDK_LIB}")
  message(FATAL_ERROR "Unitree SDK prebuilt library missing: ${UNITREE_SDK_LIB}")
endif()
message(STATUS "Using Unitree SDK library: ${UNITREE_SDK_LIB}")

# --- motor_bus_node ---
add_executable(motor_bus_node src/motor_bus_node.cpp)
target_include_directories(motor_bus_node PRIVATE
  "${UNITREE_SDK_DIR}/include"
)
ament_target_dependencies(motor_bus_node rclcpp sensor_msgs std_srvs qmini_msgs)
target_link_libraries(motor_bus_node
  ${UNITREE_SDK_LIB}
  yaml-cpp
)
install(TARGETS motor_bus_node DESTINATION lib/${PROJECT_NAME})

# Bundle the SDK runtime library so the installed node can find it.
install(FILES "${UNITREE_SDK_LIB}" DESTINATION lib)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
