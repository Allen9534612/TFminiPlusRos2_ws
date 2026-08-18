cmake_minimum_required(VERSION 3.8)
project(lidar_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(calculate_angle
  src/calculate_angle.cpp
)

ament_target_dependencies(
  calculate_angle
  rclcpp
  sensor_msgs
  message_filters
)

install(
  TARGETS calculate_angle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
