cmake_minimum_required(VERSION 3.8)
project(towtruck_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------
# Dependencies
# -----------------------
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# Autoware message dependencies (Python scripts)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

# -----------------------
# C++ composable components
# -----------------------
add_library(${PROJECT_NAME}_components SHARED
  src/livox_to_autoware_component.cpp
  src/pointcloud_ex_to_xyzirc_component.cpp
)

target_include_directories(${PROJECT_NAME}_components PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "towtruck_interface::LivoxToAutoware"
  EXECUTABLE livox_to_autoware_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "towtruck_interface::PointCloudExToXyzirc"
  EXECUTABLE pointcloud_ex_to_xyzirc_node
)

install(
  TARGETS ${PROJECT_NAME}_components
  EXPORT  export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_components sensor_msgs)

# -----------------------
# Install Python nodes (vehicle bridges + dummy perception)
# -----------------------
install(
  PROGRAMS
    scripts/serial_reader.py
    scripts/vehicle_kinematics.py
    scripts/autoware_odometry_publisher.py
    scripts/autoware_control.py
    scripts/livox_downsample.py
    scripts/dummy_perception_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------
# Install launch files
# -----------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------
# Testing
# -----------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
