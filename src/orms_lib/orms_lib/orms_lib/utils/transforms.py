import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation

def pose_to_matrix(pose: Pose) -> np.ndarray:
    """ Convert geometry_msgs Pose to 4x4 transform matrix """
    T = np.eye(4)
    T[0, 3] = pose.position.x
    T[1, 3] = pose.position.y
    T[2, 3] = pose.position.z
    r = Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    T[:3, :3] = r.as_matrix()
    return T

def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """ Convert 4x4 transform matrix to geometry_msgs Pose """
    pose = Pose()
    pose.position.x = matrix[0, 3]
    pose.position.y = matrix[1, 3]
    pose.position.z = matrix[2, 3]
    r = Rotation.from_matrix(matrix[:3, :3])
    q = r.as_quat() # x, y, z, w
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def multiply_poses(pose1: Pose, pose2: Pose) -> Pose:
    """ Multiply two poses (T1 * T2) """
    T1 = pose_to_matrix(pose1)
    T2 = pose_to_matrix(pose2)
    return matrix_to_pose(T1 @ T2)

def rp2t(rot: np.ndarray, pos: np.ndarray) -> Pose:
    """ Convert rotation matrix and position vector to Pose (MATLAB equivalent) """
    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = pos.flatten()
    return matrix_to_pose(T)
