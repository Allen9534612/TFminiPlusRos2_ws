cmake_minimum_required(VERSION 3.8)
project(tfmini_plus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(tfmini_pub
  src/tfmini_pub.cpp
)

ament_target_dependencies(
  tfmini_pub
  rclcpp
  sensor_msgs
)

install(
  TARGETS tfmini_pub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
