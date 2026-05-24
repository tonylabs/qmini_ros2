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

# --- ONNX Runtime (system-installed prebuilt; not provided by ROS) ---
# Looked up via env hint ONNXRUNTIME_DIR or default /opt/onnxruntime.
# Node code lives in M5. Build is gated until the lib is present.
if(DEFINED ENV{ONNXRUNTIME_DIR})
  set(ONNXRUNTIME_DIR $ENV{ONNXRUNTIME_DIR})
else()
  set(ONNXRUNTIME_DIR "/opt/onnxruntime")
endif()
if(EXISTS "${ONNXRUNTIME_DIR}/include/onnxruntime_cxx_api.h")
  message(STATUS "Found ONNX Runtime at ${ONNXRUNTIME_DIR}")
else()
  message(STATUS "ONNX Runtime not yet installed at ${ONNXRUNTIME_DIR} — qmini_rl will build a stub. Install before M5.")
endif()

# Install the policy ONNX into share/ so the runtime can find it.
install(
  DIRECTORY policies config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
