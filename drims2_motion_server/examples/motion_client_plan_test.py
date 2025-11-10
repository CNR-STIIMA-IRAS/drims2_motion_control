import time

import rclpy
from geometry_msgs.msg import PoseStamped
from drims2_motion_server.motion_client import MotionClient
from sympy import false


def main() -> None:
    rclpy.init()
    motion_client = MotionClient()

    # Example: plan_to_joint
    joint_goal = [
        -0.16410449298697685,    # shoulder_pan_joint
        -1.6820258857637684,     # shoulder_lift_joint
        1.6139817698694139,      # elbow_joint
        -1.8017236579269869,     # wrist_1_joint
        -1.5701870879802997,     # wrist_2_joint
        -0.16411033649582998,    # wrist_3_joint
    ]

    result, trj = motion_client.plan_to_joint(joint_target=joint_goal, joint_start=None)

    print("Plan to joint result:", result)
    print("Planned trj:", trj)

    motion_client.execute_last_planned_trajectory()

    # Example: plan_to_pose
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "tip" # relative motion wrt tip frame
    pose_msg.pose.position.x = 0.0
    pose_msg.pose.position.y = 0.0
    pose_msg.pose.position.z = 0.15
    pose_msg.pose.orientation.w = 1.0

    result, trj = motion_client.plan_to_pose(pose=pose_msg, joint_start=None, cartesian_motion=True)
    print("Plan to pose result:", result)
    print("Planned trj:", trj)

    motion_client.execute_trajectory(trj)


    rclpy.shutdown()

if __name__ == '__main__':
    main()
