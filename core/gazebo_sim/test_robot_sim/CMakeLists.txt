cmake_minimum_required(VERSION 3.8)
project(test_robot_sim)
find_package(ament_cmake REQUIRED)
ament_package()
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
