cmake_minimum_required(VERSION 3.8)
project(target_redeem_box)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17) # Use C++17
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED) # Use yaml_cpp_vendor for YAML
find_package(OpenCV REQUIRED) # Find OpenCV
find_package(PCL REQUIRED) # Find PCL
find_package(cv_bridge REQUIRED) # Find cv_bridge
find_package(pcl_conversions REQUIRED) # Find pcl_conversions
find_package(ament_index_cpp REQUIRED) # Find ament_index_cpp

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Source files
set(SOURCES
  src/DetectionOrchestratorNode.cpp
  src/RectangleDetector.cpp
  # Add other detector source files here as they are created
)

# Add executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor
  OpenCV
  PCL
  cv_bridge
  pcl_conversions
  ament_index_cpp
)

# Install files
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch file
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Install config file
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)


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

ament_package()
