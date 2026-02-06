################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(openmanipulator_task_executor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pnp_interfaces REQUIRED)

################################################################################
# Include cpp "Header" directory
################################################################################
include_directories(include)

################################################################################
# Create cpp executable
################################################################################
add_executable(robot_interface src/robot_interface.cpp)
# add_executable(robot_interface_test src/robot_interface_test.cpp)
# add_executable(simple_setposetarget src/simple_setposetarget.cpp)

################################################################################
# Export header include paths
################################################################################
# target_include_directories(robot_interface PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)

################################################################################
# Declare ROS 2/MoveIt dependencies (headers, libraries, and runtime linking)
################################################################################
ament_target_dependencies(robot_interface
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "std_srvs"
  "control_msgs"
  "geometry_msgs"
  "pnp_interfaces"
)

# ament_target_dependencies(
#   robot_interface_test
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "std_srvs"
#   "control_msgs"
#   "geometry_msgs"
#   "pnp_interfaces"
# )

# ament_target_dependencies(
#   simple_setposetarget
#   "moveit_ros_planning_interface"
#   "rclcpp"
#   "rclcpp_action"
#   "std_srvs"
#   "control_msgs"
#   "geometry_msgs"
#   "pnp_interfaces"
# )

################################################################################
# Install cpp, python executable
################################################################################
# cpp executable
install(TARGETS robot_interface
  # robot_interface_test
  # simple_setposetarget
  DESTINATION lib/${PROJECT_NAME})

# python modules
ament_python_install_package(${PROJECT_NAME})

# python executables
install(PROGRAMS
  scripts/open_manipulator_x_pickandplace.py
  scripts/keyboard_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# config files (calibration, yaml, npz, etc.)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# BUILD_TESTING
################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)

ament_package()
