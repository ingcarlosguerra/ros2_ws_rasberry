cmake_minimum_required(VERSION 3.8)

project(mentorpi_i2c_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencias ROS 2
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# ============================================================
# Ejecutable
# ============================================================

add_executable(mentorpi_i2c_motor_node
  src/mentorpi_i2c_motor_node.cpp
  src/hiwonder_emm.cpp
)

# ============================================================
# Include
# ============================================================

target_include_directories(mentorpi_i2c_motor_node
  PRIVATE
    include
)

# ============================================================
# Dependencias del ejecutable
# ============================================================

ament_target_dependencies(mentorpi_i2c_motor_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

# ============================================================
# Instalar ejecutable
# ============================================================

install(
  TARGETS mentorpi_i2c_motor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Configuración
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
