cmake_minimum_required(VERSION 3.8)
project(drims2_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(drims2_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
  rclcpp
  behaviortree_cpp
  behaviortree_ros2
  drims2_msgs
  control_msgs
  moveit_msgs
  moveit_core
  tf2_ros
  tf2
)
    
include_directories(include)

add_executable(bt_executer_node
    src/bt_executer_node.cpp
)
ament_target_dependencies(bt_executer_node ${dependencies})

add_library(move_to_pose SHARED
    src/leaves/move_to_pose.cpp
)
list(APPEND plugin_libs move_to_pose)

add_library(move_to_joint SHARED
    src/leaves/move_to_joint.cpp
)
list(APPEND plugin_libs move_to_joint)

add_library(gripper_command SHARED
    src/leaves/gripper_command.cpp
)
list(APPEND plugin_libs gripper_command)

add_library(dice_identification SHARED
    src/leaves/dice_identification.cpp
)
list(APPEND plugin_libs dice_identification)
add_library(attach_object SHARED
    src/leaves/attach_object.cpp
)
list(APPEND plugin_libs attach_object)
add_library(detach_object SHARED
    src/leaves/detach_object.cpp
)
list(APPEND plugin_libs detach_object)

add_library(check_dice_face SHARED
    src/leaves/check_dice_face.cpp
)
list(APPEND plugin_libs check_dice_face)

add_library(can_transform SHARED
    src/leaves/can_transform.cpp
)
list(APPEND plugin_libs can_transform)

foreach(bt_plugin ${plugin_libs})
  # target_link_libraries(${bt_plugin} ${dependencies})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)

endforeach()

install(DIRECTORY launch config trees DESTINATION share/${PROJECT_NAME})

install(TARGETS
  bt_executer_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
    DESTINATION include/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()