from orms_lib.interfaces.robot_base import RobotBase
import rclpy
from geometry_msgs.msg import PoseStamped
from typing import List

class UR5eRobot(RobotBase):
    def __init__(self, node: rclpy.node.Node):
        super().__init__(node)
        self.node.get_logger().info("UR5eRobot interface initialized (scaffolding).")

    def move_cartesian(self, target_pose: PoseStamped, duration: float, stop_condition=None) -> bool:
        self.node.get_logger().warn("UR5eRobot.move_cartesian not implemented.")
        return True

    def move_cartesian_path(self, waypoints: List[PoseStamped]) -> bool:
        self.node.get_logger().warn("UR5eRobot.move_cartesian_path not implemented.")
        return True

    def move_cartesian_relative(self, delta_pos: List[float], duration: float, stop_condition=None) -> bool:
        self.node.get_logger().warn("UR5eRobot.move_cartesian_relative not implemented.")
        return True

    def move_joints(self, joint_positions: List[float], duration: float) -> bool:
        self.node.get_logger().warn("UR5eRobot.move_joints not implemented.")
        return True

    def get_ee_pose(self) -> PoseStamped:
        self.node.get_logger().warn("UR5eRobot.get_ee_pose not implemented.")
        return PoseStamped()

    def get_wrench(self) -> List[float]:
        self.node.get_logger().warn("UR5eRobot.get_wrench not implemented.")
        return [0.0] * 6

    def set_speed_scaling(self, velocity: float, acceleration: float):
        self.node.get_logger().warn("UR5eRobot.set_speed_scaling not implemented.")
