cmake_minimum_required(VERSION 3.16)
project(qmini_rl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(qmini_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# --- ONNX Runtime (system-installed prebuilt; not provided by ROS) ----------
# Install an official onnxruntime release (standard include/, lib/ layout) and
# point ONNXRUNTIME_DIR at it; defaults to /opt/onnxruntime.
#   On the Pi 5: install the official aarch64 release to /opt/onnxruntime
#   (see docs/8-POLICY_RUNNER.md "ONNX Runtime").
if(DEFINED ENV{ONNXRUNTIME_DIR})
  set(ONNXRUNTIME_DIR $ENV{ONNXRUNTIME_DIR})
else()
  set(ONNXRUNTIME_DIR "/opt/onnxruntime")
endif()

find_path(ORT_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS "${ONNXRUNTIME_DIR}/include"
)
find_library(ORT_LIBRARY
  NAMES onnxruntime
  HINTS "${ONNXRUNTIME_DIR}/lib"
)

if(ORT_INCLUDE_DIR AND ORT_LIBRARY)
  message(STATUS "qmini_rl: ONNX Runtime headers ${ORT_INCLUDE_DIR}, lib ${ORT_LIBRARY}")

  add_executable(policy_runner_node src/policy_runner_node.cpp)
  target_include_directories(policy_runner_node PRIVATE "${ORT_INCLUDE_DIR}")
  ament_target_dependencies(policy_runner_node
    rclcpp sensor_msgs geometry_msgs qmini_msgs)
  target_link_libraries(policy_runner_node "${ORT_LIBRARY}" yaml-cpp)
  install(TARGETS policy_runner_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "qmini_rl: ONNX Runtime NOT found under '${ONNXRUNTIME_DIR}'. "
    "Skipping policy_runner_node. Install an onnxruntime release (include/, lib/) "
    "and set ONNXRUNTIME_DIR, then rebuild.")
endif()

# Install the policy ONNX + config + launch so the runtime can find them.
install(
  DIRECTORY policies config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
