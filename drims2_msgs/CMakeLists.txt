cmake_minimum_required(VERSION 3.8)
project(drims2_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToJoint.action"
  "action/MoveToPose.action"
  "action/PlanToJoint.action"
  "action/PlanToPose.action"
  "action/ExecutePlannedTrajectory.action"
  "srv/DiceIdentification.srv"
  "srv/AttachObject.srv"
  "srv/DetachObject.srv"
  "srv/SolveIK.srv"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()