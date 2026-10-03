cmake_minimum_required(VERSION 3.8)

project(tank_i2c_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(tank_i2c_driver_node
  src/tank_i2c_driver.cpp
)

ament_target_dependencies(
  tank_i2c_driver_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

install(
  TARGETS
    tank_i2c_driver_node
  DESTINATION
    lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

