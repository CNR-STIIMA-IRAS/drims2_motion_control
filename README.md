# Easy Motion

This repository is initially developed for **DRIMS2 Summer School** as a simplified interface for planinng and control, and provides a motion control stack based on ROS 2 and MoveIt (through Pymoveit2). It includes behavior tree integration, motion command interfaces, and an action server for handling motion requests.

## Repository Structure

This repository contains the following ROS 2 packages:

- **`easy_motion_msgs`**  
  Contains the ROS 2 interface definitions for motion commands. Key interfaces include:
  - Actions:
    - `MoveToPose`
    - `MoveToJoint`
  - Services:
    - `AttachObject`
    - `DetachObject`
    - `DiceIdentification`


- **`easy_motion`**  
  Implements the motion **action/service server** that:
  - Receives `MoveToPose` and `MoveToJoint` requests.
  - Handles `AttachObject` and `DetachObject` service calls.
  - Interfaces with **MoveIt** to plan and execute robot motions.

  🔗 [Documentation](https://cnr-stiima-iras.github.io/easy_motion/index.html)

- **`easy_motion_behavior_tree`**  
  Integrates a **Behavior Tree engine** to control task execution. This package contains:
  - Custom **leaf nodes** for motion commands.
  - Logic for behavior tree loading and execution.

## Getting Started

### Clone the Repository

```bash
mkdir -p ~/projects/easy_ws/src
cd ~/projects/easy_ws/src
git clone https://github.com/CNR-STIIMA-IRAS/easy_motion.git
vcs import < easy_motion/dependencies.repos
```
