cmake_minimum_required(VERSION 3.8)
project(go2_description)

find_package(ament_cmake REQUIRED)

# Install all resource directories so they are accessible at runtime
install(DIRECTORY
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
