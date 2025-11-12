from typing import Optional, Tuple, List
from threading import Thread

import rclpy
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node

from drims2_msgs.action import MoveToPose, MoveToJoint, PlanToPose, PlanToJoint, ExecutePlannedTrajectory
from drims2_msgs.srv import AttachObject, DetachObject
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory

from pymoveit2 import MoveIt2, MoveIt2State
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from drims2_motion_server.drims2_utils import transform_to_affine, pose_stamped_to_affine, affine_to_transform, transform_to_pose_stamped
import numpy as np

class MotionServer(Node):

    def __init__(self):
        super().__init__('motion_server_node')

        self.declare_parameter('move_group_name', 'manipulator')
        self.declare_parameter('joint_names', [''])
        self.declare_parameter('base_link_name', 'base_link')
        self.declare_parameter('end_effector_name', 'tool0')
        self.declare_parameter('planner_id', 'BiTRRT')
        self.declare_parameter('cartesian_max_step', 0.005)
        self.declare_parameter('cartesian_fraction_threshold', 0.001)
        self.declare_parameter('cartesian_jump_threshold', 0.001)
        self.declare_parameter('cartesian_avoid_collisions', True)
        self.declare_parameter("max_velocity", 0.5)
        self.declare_parameter("max_acceleration", 0.5)
        self.declare_parameter("use_move_group_action", False)
        self.declare_parameter("allowed_planning_time", 2.0)
        self.declare_parameter("tolerance_position", 0.001)
        self.declare_parameter("tolerance_orientation", 0.001)
        self.declare_parameter("max_motion_retries", 3)
        self.declare_parameter("max_ik_retries", 3)
        self.declare_parameter("ik_timeout", 20)
        self.declare_parameter("ik_avoid_collisions", False)        
        self.declare_parameter("virtual_end_effector", 'tip') # Used for better visual usage (think movements in tip frame) 

        self.move_group_name = self.get_parameter('move_group_name').get_parameter_value().string_value
        self.end_effector_name = self.get_parameter('end_effector_name').get_parameter_value().string_value
        self.base_link_name = self.get_parameter('base_link_name').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.max_motion_retries = self.get_parameter('max_motion_retries').get_parameter_value().integer_value
        self.virtual_end_effector = self.get_parameter('virtual_end_effector').get_parameter_value().string_value

        self.internal_node = Node(
            'motion_server_moveit2_internal_node', use_global_arguments=False, namespace=self.get_namespace())
        
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        try:     
            self.moveit2 = MoveIt2(
                node=self.internal_node,
                joint_names=self.joint_names,
                base_link_name=self.base_link_name,
                end_effector_name=self.end_effector_name,
                group_name=self.move_group_name,
                callback_group=self.callback_group,
                use_move_group_action=self.get_parameter('use_move_group_action').get_parameter_value().bool_value
            )
            self.moveit2.planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
            self.moveit2.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value
            self.moveit2.max_acceleration = self.get_parameter('max_acceleration').get_parameter_value().double_value
            self.moveit2.allowed_planning_time = self.get_parameter('allowed_planning_time').get_parameter_value().double_value
            self.moveit2.cartesian_avoid_collisions = self.get_parameter('cartesian_avoid_collisions').get_parameter_value().bool_value
        except RuntimeError as exception:
            raise exception

        self.move_to_pose_action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self.move_to_pose_callback,
            cancel_callback=self.move_to_pose_cancel_callback,
        )
        self.move_to_joint_action_server = ActionServer(
            self,
            MoveToJoint,
            "move_to_joint",
            execute_callback=self.move_to_joint_callback,
            cancel_callback=self.move_to_joint_cancel_callback,
        )
        self.plan_to_pose_action_server = ActionServer(
            self,
            PlanToPose,
            "plan_to_pose",
            execute_callback=self.plan_to_pose_callback,
            cancel_callback=self.plan_to_pose_cancel_callback,
        )
        self.plan_to_joint_action_server = ActionServer(
            self,
            PlanToJoint,
            "plan_to_joint",
            execute_callback=self.plan_to_joint_callback,
            cancel_callback=self.plan_to_joint_cancel_callback,
        )
        self.execute_trajectory_action_server = ActionServer(
            self,
            ExecutePlannedTrajectory,
            "execute_planned_trajectory",
            execute_callback=self.execute_planned_trajectory_callback,
            cancel_callback=self.execute_planned_trajectory_cancel_callback,
        )
        self.attach_service = self.create_service(
            AttachObject,
            'attach_object',
            self.attach_object_callback
        )
        self.detach_service = self.create_service(
            DetachObject,
            'detach_object',
            self.detach_object_callback
        )
        self.compute_ik_client = self.create_client(
            GetPositionIK,
            'compute_ik',
            callback_group=self.callback_group
        )
        if not self.compute_ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                "Compute IK service not available, cannot start motion server."
            )
            raise RuntimeError("Compute IK service not available")

        self.tf_buffer = Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.internal_util_rate = self.create_rate(10)
        self.T_ee_from_virtual = None
        self.get_logger().info("Motion server is ready to receive requests")

    def init_virtual_to_ee_transform(self) -> Optional[TransformStamped]:
        """
        Perform a TF lookup between the virtual end-effector frame and the real end-effector frame.
        Save the transform so it can be reused later without doing the lookup again.

        If the virtual frame is the same as the real end-effector frame, no offset is needed.
        """
        self.T_ee_from_virtual: Optional[None] = None
        if self.virtual_end_effector == self.end_effector_name:
            self.get_logger().info(
                "Virtual end-effector frame is the same as the end-effector frame; "
                "no offset will be applied."
            )
            return 
        
        self.T_ee_from_virtual = self._lookup_transform(
            target_frame=self.end_effector_name,
            source_frame=self.virtual_end_effector)

        if self.T_ee_from_virtual is None:
            self.get_logger().info(
                f"Failed to retrieve transform from '{self.virtual_end_effector}' "
                f"to '{self.end_effector_name}' after timeout."
            )
        return 
    
    def _lookup_transform(self, target_frame: str, source_frame: str) -> Optional[TransformStamped]:
        """
        Lookup a transform between two frames.
        Returns None if the transform is not available.
        """
        timeout = 10.0
        start_time = self.get_clock().now()
        self.get_logger().info(f"Start time: {start_time}")
        deadline = self.get_clock().now() + rclpy.duration.Duration(seconds=timeout)
        self.get_logger().info(f"Deadline: {deadline}")
        rate = self.create_rate(10)
        relative_transform = None
        while (self.get_clock().now() < deadline):
            self.get_logger().info("Waiting for virtual->EE transform...")
            try:
                # Get the transform from virtual frame to end-effector frame
                relative_transform = self.tf_buffer.lookup_transform(
                    target_frame=target_frame,
                    source_frame=source_frame,
                    time=rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                self.get_logger().info(f"Transform stored for virtual->EE offset: {self.T_ee_from_virtual}.")
                break
            except TransformException as ex:
                # Transform not available at startup; will retry later when needed
                self.get_logger().info(
                    f"Could not find TF '{self.virtual_end_effector}' -> "
                    f"'{self.end_effector_name}' at startup: {ex}. "
                    "Proceeding without offset; will retry on first goal."
                )
                rate.sleep()
        return relative_transform

    def _apply_virtual_offset(self, goal_pose: PoseStamped) -> PoseStamped:
        """
        Apply the virtual end-effector offset to the goal pose if necessary.
        If the virtual frame is the same as the end-effector frame, return the original pose.
        """
        if self.T_ee_from_virtual is None:
            return goal_pose
        
        T_base_link_referece = self._lookup_transform(
            target_frame=self.base_link_name,
            source_frame=goal_pose.header.frame_id
        )
        
        M_base_link_referece = transform_to_affine(T_base_link_referece)
        M_reference_goal = pose_stamped_to_affine(goal_pose)
        M_ee_from_virtual = transform_to_affine(self.T_ee_from_virtual)
        M_base_virtual_goal = M_base_link_referece @ M_reference_goal @ np.linalg.inv(M_ee_from_virtual)

        T_base_virtual_goal = affine_to_transform(M_base_virtual_goal, self.base_link_name, 'pose_goal_frame')

        # Apply the transform to the goal pose
        transformed_pose = transform_to_pose_stamped(T_base_virtual_goal)
        self.get_logger().info(
            f"Applying virtual offset: {self.T_ee_from_virtual.transform}"
        )
        return transformed_pose

    def move_to_pose_callback(self, goal_handle: ServerGoalHandle) -> MoveToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion
        
        goal_pose = self._apply_virtual_offset(goal_pose)
        self.broadcast_pose_goal_tf(goal_pose)  # For debugging purposes show the goal pose

        # Preparing action result
        action_result = MoveToPose.Result()

        max_motion_retries = self.get_parameter('max_motion_retries').get_parameter_value().integer_value
        if cartesian_motion:
            motion_result = self._move_to_pose_with_retries(goal_pose=goal_pose, 
                                                            cartesian=True, 
                                                            max_attempts=max_motion_retries)
            action_result.result.val = motion_result.val
        else:
            max_ik_retries = self.get_parameter('max_ik_retries').get_parameter_value().integer_value
            last_ik_result_code = MoveItErrorCodes()
            for attempt in range(max_ik_retries + 1):
                robot_configuration, ik_result_code = self._compute_ik(goal_pose)

                if ik_result_code.val == MoveItErrorCodes.SUCCESS and robot_configuration is not None:
                    self.get_logger().info(f"IK solution found: {robot_configuration}")
                    break
            else:
                last_ik_result_code.val = ik_result_code.val if ik_result_code is not None else MoveItErrorCodes.NO_IK_SOLUTION

                self.get_logger().warn(f"IK computation failed with code {last_ik_result_code.val}")
                self.get_logger().warn(f"Move to pose is aborted due to no IK solution after {max_ik_retries} attempts.")
                goal_handle.abort()
                action_result.result.val = last_ik_result_code.val
                return action_result

            motion_result = self._move_to_configuration_with_retries(robot_configuration, max_motion_retries)
            action_result.result.val = motion_result.val
        
        goal_handle.succeed()
        return action_result

    def move_to_pose_cancel_callback(self, goal_handle):
        pass

    def _move_to_configuration_with_retries(self, robot_configuration: List[float], max_attempts: int) -> MoveItErrorCodes:
        result_code = MoveItErrorCodes()
        if not robot_configuration or len(robot_configuration) != len(self.joint_names):
            self.get_logger().error("Invalid robot configuration provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"[Attempt {attempt}/{max_attempts}] Moving to configuration")

            self.moveit2.move_to_configuration(robot_configuration, self.joint_names)
            partial_result = self.moveit2.wait_until_executed()
            motion_result = self.moveit2.get_last_execution_error_code()

            self.get_logger().info(f"Partial result: {partial_result}")
            self.get_logger().info(f"Motion result: {motion_result}")

            if partial_result and motion_result and motion_result.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Motion executed successfully.")
                return motion_result

            self.get_logger().warn(f"Motion failed with code {motion_result.val if motion_result else 'N/A'}; retrying...")

        # If all attempts fail
        result_code.val = motion_result.val if motion_result and partial_result else MoveItErrorCodes.FAILURE
        return result_code

    def _plan_to_configuration_with_retries(self, target_configuration: List[float], start_configuration: List[float], max_attempts: int) \
            -> Tuple[MoveItErrorCodes, JointTrajectory]:
        result_code = MoveItErrorCodes()
        if not target_configuration or len(target_configuration) != len(self.joint_names):
            self.get_logger().error("Invalid robot target configuration provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code, None

        if len(start_configuration)!=0 and len(start_configuration) != len(self.joint_names):
            self.get_logger().error("Invalid robot start configuration provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code, None

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"[Attempt {attempt}/{max_attempts}] Planning to configuration")

            trj = self.moveit2.plan(
                joint_positions=target_configuration,
                start_joint_state=start_configuration if len(start_configuration)>0 else None, # if None, take current state
                joint_names=self.joint_names
            )

            if trj is not None:
                self.get_logger().info("Motion planned successfully.")
                result_code.val = MoveItErrorCodes.SUCCESS
                return result_code, trj

            self.get_logger().warn(f"Planning failed. Retrying...")

        # If all attempts fail
        result_code.val = MoveItErrorCodes.FAILURE
        return result_code, None


    def _move_to_pose_with_retries(self, goal_pose: PoseStamped, cartesian: bool, max_attempts: int) -> MoveItErrorCodes:
        result_code = MoveItErrorCodes()
        if not goal_pose or not isinstance(goal_pose, PoseStamped):
            self.get_logger().error("Invalid goal pose provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code
        
        cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
        cartesian_fraction_threshold = self.get_parameter('cartesian_fraction_threshold').get_parameter_value().double_value
        tolerance_position = self.get_parameter('tolerance_position').get_parameter_value().double_value
        tolerance_orientation = self.get_parameter('tolerance_orientation').get_parameter_value().double_value

        
        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"[Attempt {attempt}/{max_attempts}] Moving to configuration")


            self.moveit2.move_to_pose(
                pose=goal_pose,
                cartesian=cartesian,
                cartesian_max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation
            )
            partial_result = self.moveit2.wait_until_executed()
            motion_result = self.moveit2.get_last_execution_error_code()
            self.get_logger().info(f"Partial result: {partial_result}")
            self.get_logger().info(f"Motion result: {motion_result}")

            if partial_result and motion_result and motion_result.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info("Motion executed successfully.")
                return motion_result

            self.get_logger().warn(f"Motion failed with code {motion_result.val if motion_result else 'N/A'}; retrying...")

        # If all attempts fail
        result_code.val = motion_result.val if motion_result else MoveItErrorCodes.FAILURE
        return result_code

    def _plan_to_pose_with_retries(self, goal_pose: PoseStamped, start_configuration: List[float], cartesian: bool,
                                   max_attempts: int) -> Tuple[MoveItErrorCodes, JointTrajectory]:
        result_code = MoveItErrorCodes()
        if not goal_pose or not isinstance(goal_pose, PoseStamped):
            self.get_logger().error("Invalid goal pose provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code, None

        if len(start_configuration) != 0 and len(start_configuration) != len(self.joint_names):
            self.get_logger().error("Invalid robot start configuration provided for motion.")
            result_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return result_code, None

        cartesian_max_step = self.get_parameter('cartesian_max_step').get_parameter_value().double_value
        cartesian_fraction_threshold = self.get_parameter(
            'cartesian_fraction_threshold').get_parameter_value().double_value
        tolerance_position = self.get_parameter('tolerance_position').get_parameter_value().double_value
        tolerance_orientation = self.get_parameter('tolerance_orientation').get_parameter_value().double_value

        for attempt in range(1, max_attempts + 1):
            self.get_logger().info(f"[Attempt {attempt}/{max_attempts}] Planning to configuration")

            trj = self.moveit2.plan(
                pose=goal_pose,
                cartesian=cartesian,
                max_step=cartesian_max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation,
                start_joint_state=start_configuration if len(start_configuration)>0 else None, # if None, take current state
            )

            if trj is not None:
                self.get_logger().info("Motion planned successfully.")
                result_code.val = MoveItErrorCodes.SUCCESS
                return result_code, trj

            self.get_logger().warn(f"Planning failed. Retrying...")

        # If all attempts fail
        result_code.val = MoveItErrorCodes.FAILURE
        return result_code, None

    def _execute_planned_trajectory(self, trajectory: JointTrajectory) -> MoveItErrorCodes:
        result_code = MoveItErrorCodes()

        # check trj is valid

        self.moveit2.execute(trajectory)
        partial_result = self.moveit2.wait_until_executed()
        motion_result = self.moveit2.get_last_execution_error_code()

        self.get_logger().info(f"Partial result: {partial_result}")
        self.get_logger().info(f"Motion result: {motion_result}")

        if partial_result and motion_result and motion_result.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Motion executed successfully.")
            return motion_result

        self.get_logger().warn(f"Motion failed with code {motion_result.val if motion_result else 'N/A'}")
        result_code.val = motion_result.val if motion_result and partial_result else MoveItErrorCodes.FAILURE
        return result_code

    def _compute_ik(self, goal_pose: PoseStamped) -> Tuple[Optional[List[float]], MoveItErrorCodes]:
        """
        Compute IK for the given pose.

        Returns
        -------
        (positions, err_code)
        positions: list of joint positions ordered as self.joint_names, or None on failure
        err_code : MoveItErrorCodes instance
        """
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = self.move_group_name
        ik_req.ik_request.pose_stamped = goal_pose
        ik_req.ik_request.avoid_collisions = self.get_parameter('ik_avoid_collisions').get_parameter_value().integer_value
        ik_req.ik_request.ik_link_name = self.end_effector_name
        ik_req.ik_request.timeout.sec = self.get_parameter('ik_timeout').get_parameter_value().integer_value

        # Return code
        return_code = MoveItErrorCodes()

        # Current joint state as seed
        self.moveit2.wait_new_joint_state()
        if self.moveit2.joint_state is not None:
            ik_req.ik_request.robot_state.joint_state = self.moveit2.joint_state
        else:
            self.get_logger().error("Joint state not yet available, cannot compute IK.")
            return_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return None, return_code

        # Call the IK service
        future = self.compute_ik_client.call_async(ik_req)

        rate = self.create_rate(10)
        timeout = 3.0
        start_time = self.get_clock().now()
        while not future.done() and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            self.get_logger().info("Waiting for IK solution...")
            rate.sleep()

        if not future.done():
            self.get_logger().warn("IK service call timeout.")
            return_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return None, return_code

        try:
            response = future.result()
        except Exception as ex:
            self.get_logger().error(f"IK service call failed: {ex}")
            return_code.val = MoveItErrorCodes.NO_IK_SOLUTION
            return None, return_code

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().warn(f"IK failed with code {response.error_code.val}")
            return None, response.error_code

        # Map to self.joint_names order
        joint_state = response.solution.joint_state
        name_to_pos = dict(zip(joint_state.name, joint_state.position))
        try:
            positions = [name_to_pos[name] for name in self.joint_names]
        except KeyError as missing:
            self.get_logger().error(f"IK solution missing joint: {missing}")
            return_code.val = MoveItErrorCodes.INVALID_ROBOT_STATE
            return None, return_code

        return positions, response.error_code

    def move_to_joint_cancel_callback(self, goal_handle):
        pass

    def move_to_joint_callback(self, goal_handle: ServerGoalHandle) -> MoveToJoint.Result:
        
        joints_goal = goal_handle.request.joint_target
        self.get_logger().info(f"Moving to joint: {joints_goal}")

        motion_result = self._move_to_configuration_with_retries(joints_goal, self.max_motion_retries)
        action_result = MoveToJoint.Result()
        action_result.result.val = motion_result.val
        
        goal_handle.succeed()
        return action_result

    def execute_planned_trajectory_cancel_callback(self, goal_handle):
        pass

    def execute_planned_trajectory_callback(self, goal_handle: ServerGoalHandle) -> ExecutePlannedTrajectory.Result:
        self.get_logger().info(f"Executing trajectory...")

        motion_result = self._execute_planned_trajectory(goal_handle.request.trajectory)
        action_result = ExecutePlannedTrajectory.Result()
        action_result.result.val = motion_result.val

        goal_handle.succeed()
        return action_result

    def plan_to_joint_cancel_callback(self, goal_handle):
        pass

    def plan_to_joint_callback(self, goal_handle: ServerGoalHandle) -> PlanToJoint.Result:

        joints_goal = goal_handle.request.joint_target
        joints_start = goal_handle.request.joint_start
        if joints_start:
            self.get_logger().info(f"Planning from joint {joints_start} to {joints_goal}")
        else:
            self.get_logger().info(f"Planning from current config to {joints_goal}")

        motion_result, trj = self._plan_to_configuration_with_retries(joints_goal, joints_start, self.max_motion_retries)

        action_result = PlanToJoint.Result()
        action_result.result.val = motion_result.val
        if trj is not None:
            action_result.trajectory = trj

        goal_handle.succeed()
        return action_result

    def plan_to_pose_cancel_callback(self, goal_handle):
        pass

    def plan_to_pose_callback(self, goal_handle: ServerGoalHandle) -> PlanToPose.Result:
        goal_pose: PoseStamped = goal_handle.request.pose_target
        cartesian_motion = goal_handle.request.cartesian_motion

        joints_start = goal_handle.request.joint_start
        if joints_start:
            self.get_logger().info(f"Planning from joint {joints_start} to goal pose.")
        else:
            self.get_logger().info(f"Planning from current config to goal pose.")

        goal_pose = self._apply_virtual_offset(goal_pose)
        self.broadcast_pose_goal_tf(goal_pose)  # For debugging purposes show the goal pose

        # Preparing action result
        action_result = PlanToPose.Result()

        max_motion_retries = self.get_parameter('max_motion_retries').get_parameter_value().integer_value
        if cartesian_motion:
            motion_result, trj = self._plan_to_pose_with_retries(goal_pose=goal_pose,
                                                                start_configuration=joints_start,
                                                                cartesian=True,
                                                                max_attempts=max_motion_retries)
            action_result.result.val = motion_result.val
        else:
            max_ik_retries = self.get_parameter('max_ik_retries').get_parameter_value().integer_value
            last_ik_result_code = MoveItErrorCodes()
            for attempt in range(max_ik_retries + 1):
                robot_configuration, ik_result_code = self._compute_ik(goal_pose)

                if ik_result_code.val == MoveItErrorCodes.SUCCESS and robot_configuration is not None:
                    self.get_logger().info(f"IK solution found: {robot_configuration}")
                    break
            else:
                last_ik_result_code.val = ik_result_code.val if ik_result_code is not None else MoveItErrorCodes.NO_IK_SOLUTION

                self.get_logger().warn(f"IK computation failed with code {last_ik_result_code.val}")
                self.get_logger().warn(
                    f"Plan to pose is aborted due to no IK solution after {max_ik_retries} attempts.")
                goal_handle.abort()
                action_result.result.val = last_ik_result_code.val
                return action_result

            motion_result, trj = self._plan_to_configuration_with_retries(robot_configuration, joints_start, max_motion_retries)
            action_result.result.val = motion_result.val

        if trj is not None:
            action_result.trajectory = trj

        goal_handle.succeed()
        return action_result


    def broadcast_pose_goal_tf(self, pose_goal):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = pose_goal.header.frame_id
        t.child_frame_id = "pose_goal_frame" 

        t.transform.translation.x = pose_goal.pose.position.x
        t.transform.translation.y = pose_goal.pose.position.y
        t.transform.translation.z = pose_goal.pose.position.z

        t.transform.rotation = pose_goal.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    def attach_object_callback(self, request, response):
        self.get_logger().info(f"Attaching object '{request.object_id}' to frame '{request.target_frame_id}'")
        self.moveit2.attach_collision_object(id=request.object_id,
                                             link_name=request.target_frame_id,
                                             touch_links=[request.target_frame_id])
        attached = False
        if self.moveit2.update_planning_scene():
            for attached_obj in self.moveit2.planning_scene.robot_state.attached_collision_objects:
                if attached_obj.object.id == request.object_id:
                    attached = True

        if attached:
            self.get_logger().info(f"Object '{request.object_id}' successfully attached.")
            response.success = True
        else:
            self.get_logger().warn(f"Object '{request.object_id}' NOT found in attached objects.")
            response.success = False
                
        return response


    def detach_object_callback(self, request, response):
        self.get_logger().info(f"Detaching object '{request.object_id}'")
        self.moveit2.detach_collision_object(request.object_id)

        detached = False
        if self.moveit2.update_planning_scene():
            still_attached = any(
                attached_obj.object.id == request.object_id
                for attached_obj in self.moveit2.planning_scene.robot_state.attached_collision_objects
            )
            if not still_attached:
                detached = True

        if detached:
            self.get_logger().info(f"Object '{request.object_id}' successfully detached.")
            response.success = True
        else:
            self.get_logger().warn(f"Object '{request.object_id}' still appears attached!")
            response.success = False
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    try:
        motion_server_node = MotionServer()
    except RuntimeError as e:
        rclpy.logging.get_logger("motion_server_node").error(f"MotionServer start error: {e}")
        rclpy.shutdown()
        return
    executor.add_node(motion_server_node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    motion_server_node.init_virtual_to_ee_transform()
    spin_thread.join()
    # executor.spin()

if __name__ == '__main__':
    main()
