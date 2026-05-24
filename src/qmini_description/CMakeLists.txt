cmake_minimum_required(VERSION 3.8)
project(qmini_description)

find_package(ament_cmake REQUIRED)

# Install URDF, meshes, launch, rviz config under share/qmini_description/
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
