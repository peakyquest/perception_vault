cmake_minimum_required(VERSION 3.8)
project(navsat_pose_localizer)

# Default to C++17 if building C++ code
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###############################################################################
# Dependencies
###############################################################################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

###############################################################################
# Python Package Installation
###############################################################################

# Install Python package (makes modules importable)
ament_python_install_package(${PROJECT_NAME})

###############################################################################
# Python Executables
###############################################################################

# Install navsat_pose_localizer_node executable
install(PROGRAMS
  ${PROJECT_NAME}/navsat_pose_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME navsat_pose_localizer_node
)

###############################################################################
# Launch Files
###############################################################################

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

###############################################################################
# Testing
###############################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright check (add copyright headers to source files when ready)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (only works in git repo, add when ready)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###############################################################################
# Finalize
###############################################################################

ament_package()
