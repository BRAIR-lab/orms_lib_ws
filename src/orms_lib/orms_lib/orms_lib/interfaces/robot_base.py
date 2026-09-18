from abc import ABC, abstractmethod
from typing import List, Optional
import rclpy
from geometry_msgs.msg import PoseStamped

class RobotBase(ABC):
    """
    Abstract base class for robot hardware interfaces.
    All robot platforms (Franka, UR5e, etc.) must implement this interface.
    """
    def __init__(self, node: rclpy.node.Node):
        self.node = node

    @abstractmethod
    def move_cartesian(self, target_pose: PoseStamped, duration: float, stop_condition=None) -> bool:
        pass

    @abstractmethod
    def move_cartesian_path(self, waypoints: List[PoseStamped]) -> bool:
        pass

    @abstractmethod
    def move_cartesian_relative(self, delta_pos: List[float], duration: float, stop_condition=None) -> bool:
        pass

    @abstractmethod
    def move_joints(self, joint_positions: List[float], duration: float) -> bool:
        pass

    @abstractmethod
    def get_ee_pose(self) -> PoseStamped:
        pass

    @abstractmethod
    def get_wrench(self) -> List[float]:
        pass

    @abstractmethod
    def set_speed_scaling(self, velocity: float, acceleration: float):
        pass
