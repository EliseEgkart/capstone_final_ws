cmake_minimum_required(VERSION 3.8)
project(manipulator_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =========================================================
# Dependencies
# =========================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# =========================================================
# Hardware interface library
# =========================================================
add_library(
  manipulator_hardware
  SHARED
  src/manipulator_hardware_interface.cpp
)

target_include_directories(
  manipulator_hardware
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  manipulator_hardware
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

# =========================================================
# Pluginlib export
# =========================================================
pluginlib_export_plugin_description_file(
  hardware_interface
  manipulator_hardware.xml
)

# =========================================================
# Install
# =========================================================
install(
  TARGETS manipulator_hardware
  EXPORT export_manipulator_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES manipulator_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

# =========================================================
# Export
# =========================================================
ament_export_targets(
  export_manipulator_hardware
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()