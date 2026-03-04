cmake_minimum_required(VERSION 3.8)
project(eced3901)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)


# Create Cpp executable
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(dt1 src/eced3901_dt1.cpp)
ament_target_dependencies(dt1 rclcpp std_msgs nav_msgs geometry_msgs tf2)

add_executable(dt1_final src/dt1_final.cpp)
ament_target_dependencies(dt1_final rclcpp std_msgs nav_msgs geometry_msgs tf2 sensor_msgs)

add_executable(ultrasonic_serial_node src/ultrasonic_serial_node.cpp)
ament_target_dependencies(ultrasonic_serial_node rclcpp sensor_msgs)
target_link_libraries(ultrasonic_serial_node serial)

# Install Cpp executables
install(TARGETS
  talker
  dt1
  dt1_final
  ultrasonic_serial_node
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(
  DIRECTORY config launch maps models params rviz src worlds
  DESTINATION share/${PROJECT_NAME})


# Install Python modules
#ament_python_install_package(${PROJECT_NAME})


# Install Python executables
install(PROGRAMS
#  scripts/wp_follower.py
  scripts/demo_inspection.py
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
