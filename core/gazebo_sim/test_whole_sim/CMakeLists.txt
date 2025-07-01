cmake_minimum_required(VERSION 3.8)
project(test_whole_sim)

find_package(ament_cmake REQUIRED)


install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
