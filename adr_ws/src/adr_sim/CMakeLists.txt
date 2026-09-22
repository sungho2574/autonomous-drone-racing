cmake_minimum_required(VERSION 3.8)
project(adr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY assets config launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
