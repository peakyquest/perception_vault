cmake_minimum_required(VERSION 3.8)
project(filters)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# === Node: voxel_grid_node ===
add_executable(voxel_grid_node src/voxel_grid_node.cpp)
ament_target_dependencies(voxel_grid_node 
  rclcpp 
  sensor_msgs 
  pcl_conversions
)
target_link_libraries(voxel_grid_node ${PCL_LIBRARIES})
install(TARGETS voxel_grid_node DESTINATION lib/${PROJECT_NAME})

# === Node: ground_segmentation ===
add_executable(ground_segmentation src/ground_segmentation.cpp)
ament_target_dependencies(ground_segmentation 
  rclcpp 
  sensor_msgs 
  pcl_conversions
  visualization_msgs
)
target_link_libraries(ground_segmentation ${PCL_LIBRARIES})
install(TARGETS ground_segmentation DESTINATION lib/${PROJECT_NAME})

# === Node: outliers_filters ===
add_executable(outliers_filters src/outliers_filters.cpp)
ament_target_dependencies(outliers_filters 
  rclcpp 
  sensor_msgs 
  pcl_conversions
)
target_link_libraries(outliers_filters ${PCL_LIBRARIES})
install(TARGETS outliers_filters DESTINATION lib/${PROJECT_NAME})

# === Install launch files ===
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

