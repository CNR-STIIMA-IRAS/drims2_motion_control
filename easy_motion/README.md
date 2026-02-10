# Easy Motion

## Overview

`easy_motion` is a small ROS 2 package that provides a MoveIt2-backed motion server and a convenient client API for manipulation tasks. It exposes actions for moving the robot to Cartesian poses and joint configurations, services for attaching/detaching objects in the MoveIt planning scene, and a simple gripper action client example. The package also includes small TF/affine utilities for frame conversions and an optional "virtual end-effector" offset to simplify tool-centered motion.

---

## Features

* `MotionServer` node: integrates `pymoveit2` to plan & execute motions.

  * Actions: `move_to_pose`, `move_to_joint` (custom `easy_motion_msgs` actions).
  * Services: `attach_object`, `detach_object` (custom `easy_motion_msgs` services).
  * Supports Cartesian vs. IK-based motions, retry logic, and broadcasting a `pose_goal_frame` TF for debugging.


* `MotionClient` node: a lightweight client wrapper that:

  * Sends goals to the server (`MoveToPose`, `MoveToJoint`).
  * Calls attach/detach services.
  * Controls a gripper via `control_msgs/GripperCommand` action. Take care of the action names.

* `easy_motion_utils`: TF ↔ affine utilities and Pose/Transform helpers.

---

## Requirements

* ROS 2 (tested with Humble)
* `pymoveit2` and MoveIt 2 stack
* `easy_motion_msgs` (defines `MoveToPose`, `MoveToJoint`, `AttachObject`, `DetachObject`)

The server uses a `compute_ik` service provided by MoveIt.

---

## Installation

1. Clone the repository into your ROS 2 workspace `src/`.
2. Install Python dependencies (example using pip):

```bash
pip install -r requirements.txt
# or at least:
pip install numpy scipy pymoveit2
```

3. Build the workspace:

```bash
colcon build --packages-select easy_motion
```

4. Source the workspace:

```bash
source install/setup.bash
```

---

## Configuration (ROS parameters)

The server exposes several parameters (defaults shown in the node):

* `move_group_name` (string, default: `manipulator`)
* `joint_names` (string\[], default: `['']`)
* `base_link_name` (string, default: `base_link`)
* `end_effector_name` (string, default: `tool0`)
* `planner_id` (string, default: `BiTRRT`)
* `cartesian_max_step`, `cartesian_fraction_threshold`, `cartesian_jump_threshold` (floats)
* `cartesian_avoid_collisions` (bool)
* `max_velocity`, `max_acceleration` (floats)
* `use_move_group_action` (bool)
* `allowed_planning_time` (float)
* `tolerance_position`, `tolerance_orientation` (floats)
* `max_motion_retries`, `max_ik_retries` (ints)
* `ik_timeout` (int)
* `virtual_end_effector` (string, name of a virtual frame; default: `tip`)

You can set these via `ros2 param set` or in a launch file.

---

## Running the server

Start your MoveIt 2-related nodes (planning, controllers, joint\_state\_publisher) as required by your robot. Then launch the motion server node:

```bash
ros2 run easy_motion motion_server_node
```

If everything is configured, the server will attempt to connect to the `compute_ik` service and report readiness.

---

## Quick usage examples

### Move to a pose (action)

Send a `MoveToPose` goal using the client node or `ros2 action` CLI. Example with `ros2 action` (you need to craft a suitable goal JSON/YAML):

```bash
ros2 action send_goal /move_to_pose easy_motion_msgs/action/MoveToPose "{pose_target: {header: { frame_id: 'base_link'}, pose: { position: { x: 0.5, y: 0.0, z: 0.2 }, orientation: { x: 0, y: 0, z: 0, w: 1 }}}, cartesian_motion: false }"
```

To use Cartesian motion set `cartesian_motion: true`.

### Move to joint configuration (action)

```bash
ros2 action send_goal /move_to_joint easy_motion_msgs/action/MoveToJoint "{ joint_target: [0.0, -1.0, 1.0, 0.0, 0.0, 0.0] }"
```

### Attach / Detach object (services)

```bash
ros2 service call /attach_object easy_motion_msgs/srv/AttachObject "{ object_id: 'my_box', target_frame_id: 'ee_link' }"
ros2 service call /detach_object easy_motion_msgs/srv/DetachObject "{ object_id: 'my_box' }"
```

### Gripper command (example)

If your system exposes a gripper `control_msgs/GripperCommand` action (example name: `robotiq_action_controller/gripper_cmd`):

```bash
ros2 action send_goal /robotiq_action_controller/gripper_cmd control_msgs/action/GripperCommand "{ command: { position: 0.04, max_effort: 5.0 } }"
```

---

## API Overview

🔗 [API Website](https://cnr-stiima-iras.github.io/drims2_motion_control/index.html)

### MotionServer

* Exposes action servers for `MoveToPose` and `MoveToJoint`.
* Exposes services `attach_object` and `detach_object`.
* Uses `pymoveit2.MoveIt2` for planning and execution.
* Handles IK computation via the `compute_ik` service.
* Broadcasts a `pose_goal_frame` transform for debugging when a goal pose is received.
* Implements retry loops for both IK and motion execution.

### MotionClient

Node wrapper around action servers and services to simplify usage from Python scripts. Here you can find the available API.

<!-- --- -->

<!-- ## Virtual end-effector note

The server supports planning relative to a *virtual* end-effector frame. This is useful when you want to plan motions relative to a tool tip or a different reference than the real end-effector. The server will attempt to look up the transform between the `virtual_end_effector` frame and the configured `end_effector_name`. If the transform is not available at startup, it will retry when the first goal arrives. -->

<!-- ---

## Troubleshooting

* **Compute IK service not available**: Ensure MoveIt is running and providing the `compute_ik` service. The server will raise and exit if it cannot connect at start.
* **No joint state**: The IK computation uses the current joint state as a seed. Make sure `/joint_states` is published.
* **Motion failures**: The server retries motions a configurable number of times. Inspect the node logs to see the `MoveItErrorCodes` returned from execution. -->

---

## Contributing

Contributions and bug reports are welcome. Open issues or pull requests in the repository.

