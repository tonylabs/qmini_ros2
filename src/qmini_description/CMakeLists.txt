cmake_minimum_required(VERSION 3.8)
project(qmini_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, rviz config under share/qmini_description/.
# OPTIONAL keeps colcon working when one of these dirs is empty after a fresh
# clone (git doesn't track empty directories — launch/ is currently empty).
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
