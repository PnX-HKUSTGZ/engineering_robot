cmake_minimum_required(VERSION 3.8)
project(engineering_resolve)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_compile_options(-O2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)


include_directories("/opt/ros/humble/include/**")
include_directories("/usr/include/pcl-1.12/")
include_directories("/usr/include/eigen3/")
include_directories("/usr/include/opencv4/")
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/../../**)  # Add project root directory

# find dependencies
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pcl_conversions REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED) # 如果使用 PlanningInterface
find_package(moveit_visual_tools REQUIRED) # 如果使用可视化工具

find_package(geometric_shapes REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(command_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")

message(STATUS "SOURCES: ${SOURCES}")

add_executable(engineering_resolve ${SOURCES})

ament_target_dependencies(engineering_resolve
  geometry_msgs
  rclcpp
  OpenCV
  sensor_msgs
  cv_bridge
  rclcpp_components
  Eigen3
  tf2_ros
  pcl_conversions
  tf2
  tf2_sensor_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  geometric_shapes
  moveit_msgs
  command_interfaces
  tf2_geometry_msgs
  ament_index_cpp
  
)

target_link_libraries(engineering_resolve
  yaml-cpp::yaml-cpp
)

install(TARGETS engineering_resolve DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})


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
