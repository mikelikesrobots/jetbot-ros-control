cmake_minimum_required(VERSION 3.8)
project(jetbot_control)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jetbot_control
  src/jetbot_control.cpp
  src/i2c_device.cpp
  src/motor.cpp
)
target_include_directories(jetbot_control PUBLIC include)
ament_target_dependencies(jetbot_control rclcpp std_msgs)

install(TARGETS
  jetbot_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()
