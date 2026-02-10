# EasyMotion Behavior Tree

The `easy_motion_behavior_tree` package provides custom Behavior Tree (BT) nodes for controlling robot motions in the **EasyMotion** stack.  
It builds upon the BehaviorTree.ROS2/.CPP library and ROS 2 action interfaces to enable modular and flexible behavior orchestration.

## Overview

This package integrates motion-related actions into behavior trees, offering ready-to-use leaf nodes that interact with ROS 2 action servers such as motion planners and gripper controllers.

Each node can be loaded as a plugin and used in XML-based behavior tree definitions.
<!-- 
## Available Behavior Tree Nodes

The following custom BT **action nodes** are available:

| Node Name         | Input Ports                                                                 | Description                                                |
|-------------------|------------------------------------------------------------------------------|------------------------------------------------------------|
| `GripperCommand`  | `position` (double), `max_effort` (double)                                  | Sends a gripper command with target position and max effort. |
| `MoveToJoint`     | `joint_target` (vector\<double>)                                             | Moves the robot to a specified joint configuration.         |
| MoveToPose   | `pose_target` (PoseStamped) **or**<br>`frame_id` (string), `position` (vector<double>, size=3), `orientation` (vector<double>, size=4),<br>`cartesian_motion` (bool, optional, default: false) | Moves the robot to a target pose. Accepts either a full `pose_target` or pose components. The optional `cartesian_motion` flag enables Cartesian path planning when set to true. |
| DiceIdentification | –                                                                                                                   | `face_number` (int), `pose` (PoseStamped)                            | Calls a service to identify a die in the scene, returning the recognized face number and pose. |
 -->
## Available Behavior Tree Nodes

<!-- The following custom BT **action nodes** are available:

| Node Name          | Input Ports                                                                                                         | Output Ports                                                  | Description                                                                                     |
|--------------------|----------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------|-------------------------------------------------------------------------------------------------|
| `GripperCommand`   | `position` (double), `max_effort` (double)                                                                          | –                                                             | Sends a gripper command with target position and max effort.                                   |
| `MoveToJoint`      | `joint_target`             <GripperCommand action_name="/robotiq_action_controller/gripper_cmd" position="0.79" max_effort="1.0"/>
(vector<double>)                                                                                     | –                                                             | Moves the robot to a specified joint configuration.                                             |
| `MoveToPose`       | `pose_target` (PoseStamped) **or**<br>`frame_id` (string), `position` (vector<double>, size=3), `orientation` (vector<double>, size=4),<br>`cartesian_motion` (bool, optional, default: false) | –                                                             | Moves the robot to a target pose. Accepts either a full `pose_target` or pose components. The optional `cartesian_motion` flag enables Cartesian path planning when set to true. |
| `DiceIdentification` | –                                                                                                                   | `face_number` (int), `pose` (PoseStamped)                    | Calls a service to identify a die in the scene, returning the recognized face number and pose.  | -->

The following custom BT **action nodes** are available:

| Node Name            | Input Ports                                                                                                         | Output Ports                          | Description                                                                                     |
|----------------------|----------------------------------------------------------------------------------------------------------------------|---------------------------------------|-------------------------------------------------------------------------------------------------|
| `GripperCommand`     | `position` (double), `max_effort` (double)                                                                          | –                                     | Sends a gripper command with target position and max effort.                                   |
| `MoveToJoint`        | `joint_target` (vector<double>)                                                                                     | –                                     | Moves the robot to a specified joint configuration.                                             |
| `MoveToPose`         | `pose_target` (PoseStamped) **or**<br>`frame_id` (string), `position` (vector<double>, size=3), `orientation` (vector<double>, size=4), `cartesian_motion` (bool, optional) | –                                     | Moves the robot to a target pose. Accepts either a full `pose_target` or pose components. The optional `cartesian_motion` flag enables Cartesian path planning when set to true.                 |
| `AttachObject`       | `object_id` (string), `target_frame_id` (string)                                                                    | –                                     | Calls a service to attach a collision object to a link.                                        |
| `DetachObject`       | `object_id` (string)                                                                                                | –                                     | Calls a service to detach a collision object.                                                  |
| `DiceIdentification` | –                                                                                                                   | `face_number` (int), `pose` (PoseStamped) | Calls a service to identify a die, returning the recognized face number and pose.              |

> **Note**: Each node requires an `action_name` or `service_name` parameter to be specified in the XML tree definition. These are not listed among the input ports as they are typically constant for the node.
