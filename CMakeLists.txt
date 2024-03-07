cmake_minimum_required(VERSION 3.8)
project(jetbot_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(${PROJECT_NAME}
  SHARED
  hardware/src/jetbot_system.cpp
  hardware/src/i2c_device.cpp
  hardware/src/motor.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/jetbot_control>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "JETBOT_CONTROL_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface jetbot_control.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/jetbot_control
)
install(
  DIRECTORY description/ros2_control description/urdf
  DESTINATION share/jetbot_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/jetbot_control
)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT export_jetbot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_jetbot_system hardware/test/test_jetbot_system.cpp)
  target_include_directories(test_jetbot_system PRIVATE include)
  target_link_libraries(test_jetbot_system ${PROJECT_NAME})
  ament_target_dependencies(test_jetbot_system
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_targets(export_jetbot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
