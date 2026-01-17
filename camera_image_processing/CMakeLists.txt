cmake_minimum_required(VERSION 3.8)
project(camera_image_processing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Detect ROS distro and define macros
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  message(STATUS "ROS_DISTRO detected: ${ROS_DISTRO}")
  
  if(ROS_DISTRO STREQUAL "jazzy")
    add_compile_definitions(ROS_DISTRO_JAZZY)
    message(STATUS "ROS_DISTRO_JAZZY macro defined")
  endif()
else()
  message(WARNING "ROS_DISTRO environment variable not set")
endif()

find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Add node image_rectifier
add_executable(image_rectifier src/image_rectifier.cpp)

target_include_directories(image_rectifier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(
  image_rectifier
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "image_transport"
)

target_link_libraries(image_rectifier
  cv_bridge::cv_bridge
  ${OpenCV_LIBS}
)

install(TARGETS image_rectifier
  DESTINATION lib/${PROJECT_NAME})

ament_package()
