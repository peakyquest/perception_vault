cmake_minimum_required(VERSION 3.8)
project(camera_rtsp_streamer)

# find dependencies
find_package(ament_cmake REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python node executable
install(PROGRAMS
  ${PROJECT_NAME}/camera_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME camera_node
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
