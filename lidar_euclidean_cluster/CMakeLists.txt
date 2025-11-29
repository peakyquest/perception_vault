cmake_minimum_required(VERSION 3.8)
project(lidar_euclidean_cluster)

# Default to C++17
if(CMAKE_CXX_STANDARD EQUAL 0)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip copyright linter (uncomment when copyright is added to source files)
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint (uncomment when package is in git repo and copyright is added)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================================
# Executables
# ============================================================================
add_executable(lidar_euclidean_cluster_node 
  src/lidar_euclidean_cluster_node.cpp
)

target_include_directories(lidar_euclidean_cluster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(lidar_euclidean_cluster_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  lidar_euclidean_cluster_node
  rclcpp
  sensor_msgs
  visualization_msgs
  pcl_conversions
)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS lidar_euclidean_cluster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
