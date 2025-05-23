cmake_minimum_required(VERSION 3.8)
project(engineering_robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(serial_driver REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(command_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES "src/*.cpp")

message(${SOURCES})

add_library(engineering_robot_hardware_interface SHARED ${SOURCES})

ament_target_dependencies(engineering_robot_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  serial_driver
  command_interfaces
)

target_include_directories(engineering_robot_hardware_interface
  PUBLIC
  include
)

ament_export_include_directories(
  include
)

pluginlib_export_plugin_description_file(hardware_interface engineering_robot_hardware_interface.xml)

install(FILES
  engineering_robot_hardware_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS engineering_robot_hardware_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_libraries(engineering_robot_hardware_interface)

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
