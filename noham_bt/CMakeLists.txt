cmake_minimum_required(VERSION 3.8)
project(noham_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Find BehaviorTree.CPP 4.6 (system installation)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(isaac_ros_apriltag_interfaces REQUIRED)
find_package(noham_custom_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Create executables
add_executable(spot_bt src/spot_bt.cpp src/behavior_tree_nodes.cpp)
ament_target_dependencies(spot_bt
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  behaviortree_cpp
)

add_executable(forklift_bt src/forklift_bt.cpp src/behavior_tree_nodes.cpp src/forklift_action_nodes.cpp)
ament_target_dependencies(forklift_bt
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  behaviortree_cpp
  noham_custom_interfaces
)

add_executable(lifter_action src/lifter_action.cpp)
ament_target_dependencies(lifter_action
  rclcpp
  rclcpp_action
  sensor_msgs
  trajectory_msgs
  noham_custom_interfaces
)

add_executable(forklift_approach src/forklift_approach.cpp)
ament_target_dependencies(forklift_approach
  rclcpp
  rclcpp_action
  sensor_msgs
  isaac_ros_apriltag_interfaces
  geometry_msgs
  noham_custom_interfaces
)

add_executable(noham_actions src/noham_actions.cpp)
ament_target_dependencies(noham_actions
  rclcpp
  rclcpp_action
  sensor_msgs
  isaac_ros_apriltag_interfaces
  geometry_msgs
  trajectory_msgs
  noham_custom_interfaces
)

add_executable(manipulator_control src/manipulator_control.cpp)
ament_target_dependencies(manipulator_control
  rclcpp
  rclcpp_action
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  noham_custom_interfaces
  moveit_ros_planning_interface
  tf2_ros
  tf2
  isaac_ros_apriltag_interfaces
)

add_executable(mobile_manipulator_bt src/mobile_manipulator_bt.cpp src/behavior_tree_nodes.cpp src/mobile_manipulator_nodes.cpp)
ament_target_dependencies(mobile_manipulator_bt
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
  behaviortree_cpp
  noham_custom_interfaces
)

# Install executables
install(TARGETS
  spot_bt
  forklift_bt
  lifter_action
  forklift_approach
  noham_actions
  manipulator_control
  mobile_manipulator_bt
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

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
