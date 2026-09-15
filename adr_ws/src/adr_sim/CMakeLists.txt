cmake_minimum_required(VERSION 3.8)
project(adr_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models worlds px4 config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# 설치된 models/worlds 를 GZ_SIM_RESOURCE_PATH 에 추가 (source install/setup.bash 시 적용)
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
