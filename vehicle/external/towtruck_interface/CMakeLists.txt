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

# Autoware message dependencies
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)

# -----------------------
# Install Python nodes
# -----------------------
install(
  PROGRAMS
    scripts/serial_reader.py
    scripts/vehicle_kinematics.py
    scripts/autoware_odometry_publisher.py
    scripts/autoware_control.py
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
