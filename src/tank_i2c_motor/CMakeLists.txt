
cmake_minimum_required(VERSION 3.8)

project(tank_i2c_motor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(tank_i2c_motor_node
  src/tank_i2c_motor.cpp
)

ament_target_dependencies(
  tank_i2c_motor_node
  rclcpp
  geometry_msgs
)

install(
  TARGETS
    tank_i2c_motor_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
