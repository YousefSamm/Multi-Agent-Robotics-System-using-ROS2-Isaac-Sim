cmake_minimum_required(VERSION 3.8)
project(noham_custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ForkliftApproach.action"
  "action/LifterAction.action"
  "action/ArmPlanning.action"
  "action/GripperAction.action"
  "action/MobileManipulatorApproach.action"
  DEPENDENCIES
)

ament_package()
