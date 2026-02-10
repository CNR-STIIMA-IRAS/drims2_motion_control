# from rclpy.node import Node
# from moveit.planning import MoveItPy, PlanRequestParameters
from geometry_msgs.msg import (PoseStamped, TransformStamped)
from scipy.spatial.transform import Rotation as R
import numpy as np
from typing import Optional, Iterable


# class CartesianPlannerParams:
#     PIPELINE = 'cartesian_planner_config.planning_pipeline'
#     PLANNER_ID = 'cartesian_planner_config.planner_id'
#     PLANNING_TIME = 'cartesian_planner_config.planning_time'
#     MAX_VEL_SCALING = 'cartesian_planner_config.max_velocity_scaling_factor'
#     MAX_ACC_SCALING = 'cartesian_planner_config.max_acceleration_scaling_factor'

#     @staticmethod
#     def get_plan_request_parameters(node: Node, moveit_core: MoveItPy) -> PlanRequestParameters:

#         plan_request_param = PlanRequestParameters(moveit_core)
#         plan_request_param.planning_pipeline = node.get_parameter(CartesianPlannerParams.PIPELINE).get_parameter_value().string_value
#         plan_request_param.planner_id = node.get_parameter(CartesianPlannerParams.PLANNER_ID).get_parameter_value().string_value
#         plan_request_param.planning_time = node.get_parameter(CartesianPlannerParams.PLANNING_TIME).get_parameter_value().double_value
#         plan_request_param.max_velocity_scaling_factor = node.get_parameter(CartesianPlannerParams.MAX_VEL_SCALING).get_parameter_value().double_value
#         plan_request_param.max_acceleration_scaling_factor = node.get_parameter(CartesianPlannerParams.MAX_ACC_SCALING).get_parameter_value().double_value

#         return plan_request_param

def quat_to_rot_xyzw(q):
    """Quaternion [x, y, z, w] -> 3x3 rotation matrix (SciPy order)."""
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    return r.as_matrix()

def transform_to_affine(transform: TransformStamped) -> np.ndarray:
    """
    Convert a ROS2 TransformStamped message into a 4x4 affine transformation matrix.

    Parameters
    ----------
    transform : TransformStamped
        The ROS2 TransformStamped message containing translation and rotation.

    Returns
    -------
    np.ndarray
        4x4 affine transformation matrix where:
        - The top-left 3x3 block is the rotation matrix.
        - The top-right 3x1 column is the translation vector.
        - The last row is [0, 0, 0, 1].
    """

    transform = transform.transform
    transform_rotation_matrix = [
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w
    ]
    transform_translation = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z
    ]
    return build_affine(transform_translation, transform_rotation_matrix)

def pose_stamped_to_affine(pose: PoseStamped) -> np.ndarray:
    """
    Convert a PoseStamped to a 4x4 affine transformation matrix.

    Parameters
    ----------
    pose : PoseStamped
        The pose to convert.

    Returns
    -------
    np.ndarray
        4x4 affine transformation matrix, with rotation in the top-left 3x3
        and translation in the last column.
    """
    # Extract translation [x, y, z]
    t = pose.pose.position
    translation: Iterable[float] = [t.x, t.y, t.z]

    # Extract quaternion in [x, y, z, w] order
    q = pose.pose.orientation
    rotation_xyzq: Iterable[float] = [q.x, q.y, q.z, q.w]

    # Build the affine transformation matrix
    return build_affine(translation, rotation_xyzq)

def affine_to_transform(
    affine: np.ndarray,
    frame_id: str,
    child_frame_id: str
) -> TransformStamped:
    """
    Convert a 4x4 affine transformation matrix into a ROS2 TransformStamped.

    Parameters
    ----------
    affine : np.ndarray
        4x4 transformation matrix. The top-left 3x3 block is the rotation,
        the top-right 3x1 column is the translation.
    frame_id : str
        The name of the parent frame (TransformStamped.header.frame_id).
    child_frame_id : str
        The name of the child frame (TransformStamped.child_frame_id).

    Returns
    -------
    TransformStamped
        ROS2 TransformStamped message containing the given transform.
    """
    if affine.shape != (4, 4):
        raise ValueError(f"Expected affine shape (4,4), got {affine.shape}")

    # Extract translation [x, y, z]
    translation = affine[:3, 3]

    # Extract rotation matrix and convert to quaternion [x, y, z, w]
    rot_matrix = affine[:3, :3]
    qx, qy, qz, qw = R.from_matrix(rot_matrix).as_quat()

    # Fill the TransformStamped message
    tf_msg = TransformStamped()
    tf_msg.header.frame_id = frame_id
    tf_msg.child_frame_id = child_frame_id

    tf_msg.transform.translation.x = float(translation[0])
    tf_msg.transform.translation.y = float(translation[1])
    tf_msg.transform.translation.z = float(translation[2])

    tf_msg.transform.rotation.x = float(qx)
    tf_msg.transform.rotation.y = float(qy)
    tf_msg.transform.rotation.z = float(qz)
    tf_msg.transform.rotation.w = float(qw)

    return tf_msg



def build_affine(
        translation: Optional[Iterable] = None,
        rotation: Optional[Iterable] = None) -> np.ndarray:
    """
    Build an affine matrix from a quaternion and a translation.

    :param rotation: The quaternion as [w, x, y, z]
    :param translation: The translation as [x, y, z]
    :returns: The quaternion and the translation array
    """
    affine = np.eye(4)
    if rotation is not None:
        affine[:3, :3] = quat_to_rot_xyzw(np.asarray(rotation))
    if translation is not None:
        affine[:3, 3] = np.asarray(translation)
    return affine


def transform_to_pose_stamped(transform: TransformStamped) -> PoseStamped:
    """
    Convert a TransformStamped to a PoseStamped.

    Parameters
    ----------
    transform : TransformStamped
        The transform to convert.

    Returns
    -------
    PoseStamped
        The converted pose.
    """
    pose = PoseStamped()
    pose.header = transform.header
    pose.pose.position.x = transform.transform.translation.x
    pose.pose.position.y = transform.transform.translation.y
    pose.pose.position.z = transform.transform.translation.z

    pose.pose.orientation.x = transform.transform.rotation.x
    pose.pose.orientation.y = transform.transform.rotation.y
    pose.pose.orientation.z = transform.transform.rotation.z
    pose.pose.orientation.w = transform.transform.rotation.w

    return pose