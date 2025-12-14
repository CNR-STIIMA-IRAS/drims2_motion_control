import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from typing import Tuple, Optional, List

from drims2_msgs.action import MoveToPose, MoveToJoint, PlanToPose, PlanToJoint, ExecutePlannedTrajectory
from drims2_msgs.srv import AttachObject, DetachObject, GetIK, GetFK

from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from moveit_msgs.msg import MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory

class MotionClient(Node):
    """ROS 2 Client for controlling robot motion and gripper.

    This class provides a simplified interface for sending commands
    to the robot via action servers and services. Supported operations:
    - Move to a target pose
    - Move to joint configurations
    - Attach/detach objects
    - Move the gripper
    """


    def __init__(self,
                 move_to_pose_action_name:str ='move_to_pose',
                 move_to_joint_action_name:str ='move_to_joint',
                 plan_to_pose_action_name:str ='plan_to_pose',
                 plan_to_joint_action_name:str ='plan_to_joint',
                 execute_planned_trajectory_action_name ='execute_planned_trajectory',
                 gripper_action_name:str ='/gripper_action_controller/gripper_cmd'):
        super().__init__('motion_client_node', use_global_arguments=False)
        """Initialize the MotionClient.

        Args:
            move_to_pose_action_name (str): Action server name for moving to a pose.
            move_to_joint_action_name (str): Action server name for moving to joint positions.
            gripper_action_name (str): Action server name for controlling the gripper.

        Raises:
            RuntimeError: If one of the required action servers or services is not available.
        """


        self.declare_parameter('move_to_pose_action_name', move_to_pose_action_name)
        self.declare_parameter('move_to_joint_action_name', move_to_joint_action_name)
        self.declare_parameter('plan_to_pose_action_name', plan_to_pose_action_name)
        self.declare_parameter('plan_to_joint_action_name', plan_to_joint_action_name)
        self.declare_parameter('execute_planned_trajectory_action_name', execute_planned_trajectory_action_name)
        self.declare_parameter('gripper_action_name', gripper_action_name)

        move_to_pose_action_name = self.get_parameter('move_to_pose_action_name').get_parameter_value().string_value
        move_to_joint_action_name = self.get_parameter('move_to_joint_action_name').get_parameter_value().string_value
        plan_to_pose_action_name = self.get_parameter('plan_to_pose_action_name').get_parameter_value().string_value
        plan_to_joint_action_name = self.get_parameter('plan_to_joint_action_name').get_parameter_value().string_value
        execute_planned_trajectory_action_name = self.get_parameter('execute_planned_trajectory_action_name').get_parameter_value().string_value
        gripper_action_name = self.get_parameter('gripper_action_name').get_parameter_value().string_value

        self.move_to_pose_client = ActionClient(self, MoveToPose, move_to_pose_action_name)
        self.move_to_joint_client = ActionClient(self, MoveToJoint, move_to_joint_action_name)
        self.plan_to_pose_client = ActionClient(self, PlanToPose, plan_to_pose_action_name)
        self.plan_to_joint_client = ActionClient(self, PlanToJoint, plan_to_joint_action_name)
        self.execute_planned_trajectory_client = ActionClient(self, ExecutePlannedTrajectory, execute_planned_trajectory_action_name)
        self.gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
        self.attach_object_client = self.create_client(AttachObject, 'attach_object')
        self.detach_object_client = self.create_client(DetachObject, 'detach_object')
        self.get_ik_client = self.create_client(GetIK, 'get_ik')
        self.get_fk_client = self.create_client(GetFK, 'get_fk')

        self.last_planned_trj: JointTrajectory = None

        if not self.move_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToPose action server not available")
        if not self.move_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveToJoint action server not available")
        if not self.plan_to_pose_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("PlanToPose action server not available")
        if not self.plan_to_joint_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("PlanToJoint action server not available")
        if not self.execute_planned_trajectory_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("ExecutePlannedTrajectory action server not available")
        if not self.attach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("AttachObject service not available")
        if not self.detach_object_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("DetachObject service not available")
        if not self.gripper_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("GripperCommand action server not available")

    def move_to_pose(self, pose: PoseStamped, cartesian_motion: bool = False, relative_motion = False,
                     velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0) -> MoveItErrorCodes:
        """Move the robot to a target pose.

        Args:
            pose (PoseStamped): Target pose for the robot.
            cartesian_motion (bool, optional): If True, uses Cartesian trajectories. Defaults to False.
            relative_motion (bool, optional): If True, consider pose as a displacement from the robot initial state. Defaults to False.
            velocity_scaling (float): Velocity scaling for planning the trajectory. Will be multiplied by max_velocity
            acceleration_scaling (float): Acceleration scaling for planning the trajectory. Will be multiplied by max_acceleration

        Returns:
            MoveItErrorCodes: Result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.move_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToPose action server not available")

        goal_msg = MoveToPose.Goal()
        goal_msg.pose_target = pose
        goal_msg.cartesian_motion = cartesian_motion
        goal_msg.relative_motion = relative_motion
        goal_msg.max_velocity_scaling = velocity_scaling
        goal_msg.max_acceleration_scaling = acceleration_scaling

        future = self.move_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_pose was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        return result_future.result().result.result

    def move_to_joint(self, joint_positions: list[float], velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0) \
            -> MoveItErrorCodes:
        """Move the robot to a specific joint configuration.

        Args:
            joint_positions (list[float]): List of target joint values.
            velocity_scaling (float): Velocity scaling for planning the trajectory. Will be multiplied by max_velocity
            acceleration_scaling (float): Acceleration scaling for planning the trajectory. Will be multiplied by max_acceleration
        Returns:
            MoveItErrorCodes: Result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.move_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToJoint action server not available")

        goal_msg = MoveToJoint.Goal()
        goal_msg.joint_target = joint_positions
        goal_msg.max_velocity_scaling = velocity_scaling
        goal_msg.max_acceleration_scaling = acceleration_scaling

        future = self.move_to_joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to move_to_joint was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.result

    def plan_to_pose(self, pose: PoseStamped, joint_start: list[float] = None,
                     cartesian_motion: bool = False, relative_motion: bool = False,
                     velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0) \
            -> Tuple[MoveItErrorCodes, JointTrajectory]:
        """Move the robot to a target pose.

        Args:
            pose (PoseStamped): Target pose for the robot.
            joint_start (list[float]): List of start joint values. If None, use current move_group config.
            cartesian_motion (bool, optional): If True, uses Cartesian trajectories. Defaults to False.
            relative_motion (bool, optional): If True, consider pose as a displacement from the robot initial state. Defaults to False.
            velocity_scaling (float): Velocity scaling for planning the trajectory. Will be multiplied by max_velocity
            acceleration_scaling (float): Acceleration scaling for planning the trajectory. Will be multiplied by max_acceleration
        Returns:
            MoveItErrorCodes: Result code returned by the motion planner.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.move_to_pose_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("MoveToPose action server not available")

        goal_msg = PlanToPose.Goal()
        goal_msg.pose_target = pose
        goal_msg.cartesian_motion = cartesian_motion
        goal_msg.relative_motion = relative_motion
        goal_msg.max_velocity_scaling = velocity_scaling
        goal_msg.max_acceleration_scaling = acceleration_scaling
        if joint_start is not None:
            goal_msg.joint_start = joint_start

        future = self.plan_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to plan_to_pose was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().result.result.val == MoveItErrorCodes.SUCCESS:
            self.last_planned_trj = result_future.result().result.trajectory

        return result_future.result().result.result, result_future.result().result.trajectory

    def plan_to_joint(self, joint_target: list[float], joint_start: list[float] = None,
                      velocity_scaling: float = 1.0, acceleration_scaling: float = 1.0) -> Tuple[MoveItErrorCodes, JointTrajectory]:
        """Move the robot to a specific joint configuration.

        Args:
            joint_target (list[float]): List of target joint values.
            joint_start (list[float]): List of start joint values. If None, use current move_group config.
            velocity_scaling (float): Velocity scaling for planning the trajectory. Will be multiplied by max_velocity
            acceleration_scaling (float): Acceleration scaling for planning the trajectory. Will be multiplied by max_acceleration

        Returns:
            MoveItErrorCodes: Result code returned by the motion planner
            JointTrajectory: Planned trajectory. None if plan failed.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.plan_to_joint_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("PlanToJoint action server not available")

        goal_msg = PlanToJoint.Goal()
        goal_msg.joint_target = joint_target
        goal_msg.max_velocity_scaling = velocity_scaling
        goal_msg.max_acceleration_scaling = acceleration_scaling
        if joint_start is not None:
            goal_msg.joint_start = joint_start

        future = self.plan_to_joint_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to plan_to_joint was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        if result_future.result().result.result.val == MoveItErrorCodes.SUCCESS:
            self.last_planned_trj = result_future.result().result.trajectory

        return result_future.result().result.result, result_future.result().result.trajectory

    def execute_last_planned_trajectory(self):
        """Execute the last trajectory successfully planned by plan_to_joint() or plan_to_pose()

        Args:

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if self.last_planned_trj is None:
            raise RuntimeError("No existing trajectory available.")
        return self.execute_trajectory(self.last_planned_trj)


    def execute_trajectory(self, trajectory: JointTrajectory):
        """Execute a given JointTrajectory

        Args:
            trajectory (JointTrajectory): the trj to be executed

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.execute_planned_trajectory_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("ExecutePlannedTrajectory action server not available")

        goal_msg = ExecutePlannedTrajectory.Goal()
        goal_msg.trajectory = trajectory

        future = self.execute_planned_trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to execute_planned_trajectory was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        return result_future.result().result.result

    def attach_object(self, object_id: str, target_frame_id: str) -> bool:
        """Attach an object to the robot (e.g., to the gripper).

        Args:
            object_id (str): ID of the object in the MoveIt scene.
            target_frame_id (str): Frame of the robot to which the object should be attached.

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.attach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("AttachObject service not available")

        request = AttachObject.Request()
        request.object_id = object_id
        request.target_frame_id = target_frame_id

        future = self.attach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
    
    def detach_object(self, object_id: str) -> bool:
        """Detach an object from the robot.

        Args:
            object_id (str): ID of the object to detach.

        Returns:
            bool: True if the operation succeeded, False otherwise.

        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.detach_object_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("DetachObject service not available")

        request = DetachObject.Request()
        request.object_id = object_id

        future = self.detach_object_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def get_ik(self, pose: PoseStamped, seed: Optional[List[float]]=None) -> Tuple[MoveItErrorCodes, List[float]]:
        """Compute the inverse kinematics for the given pose

        Args:
            seed: preferred joint state
            pose (PoseStamped): Pose of the robot.

        Returns:
            MoveItErrorCodes: Result code returned by the IK solver
            List[float]: IK solution.
        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.get_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("GetIK service not available")

        request = GetIK.Request()
        request.pose  = pose
        if seed is not None:
            request.seed = seed
        future = self.get_ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().result, future.result().ik_solution


    def get_fk(self, joint_state: List[float]) -> Tuple[MoveItErrorCodes, PoseStamped]:
        """Compute the forward kinematics for the given joint state

        Args:
            joint_state: Joint state to compute FK
        Returns:
            MoveItErrorCodes: Result code returned by the FK service
            PoseStamped: end-effector pose corresponding to joint state.
        Raises:
            RuntimeError: If the service is not available.
        """
        if not self.get_fk_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("GetFK service not available")

        request = GetFK.Request()
        request.joint_state  = joint_state
        future = self.get_fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().result, future.result().pose

    def gripper_command(self, 
                        position: float, 
                        max_effort: float = 0.0) -> Tuple[bool, bool]:
        """Send a command to the gripper, i.e, move it to a specific position.

        Args:
            position (float): Target position of the gripper (units depend on controller configuration).
            max_effort (float, optional): Maximum force applied. Defaults to 0.0 (it's enough most of the time).

        Returns:
            Tuple[bool, bool]:
                - reached_goal (bool): True if the gripper reached the target position.
                - stalled (bool): True if the gripper stalled during execution.

        Raises:
            RuntimeError: If the action server is not available or the goal was rejected.
        """
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("GripperCommand action server not available")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(max_effort)

        future = self.gripper_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Goal to gripper_command was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        reached_goal = result_future.result().result.reached_goal
        stalled = result_future.result().result.stalled

        return reached_goal, stalled