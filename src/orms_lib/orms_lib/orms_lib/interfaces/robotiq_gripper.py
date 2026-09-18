from orms_lib.interfaces.gripper_base import GripperBase
import rclpy

class RobotiqGripper(GripperBase):
    def __init__(self, node: rclpy.node.Node):
        super().__init__(node)
        self.node.get_logger().info("RobotiqGripper interface initialized (scaffolding).")

    def move(self, width: float, speed: float = 0.1) -> bool:
        self.node.get_logger().warn("RobotiqGripper.move not implemented.")
        return True

    def close(self, width: float = 0.0, speed: float = 0.1, force: float = 10.0, 
              epsilon_inner: float = 0.005, epsilon_outer: float = 0.005) -> bool:
        self.node.get_logger().warn("RobotiqGripper.close not implemented.")
        return True

    def open(self) -> bool:
        self.node.get_logger().warn("RobotiqGripper.open not implemented.")
        return True

    def get_width(self) -> float:
        self.node.get_logger().warn("RobotiqGripper.get_width not implemented.")
        return 0.0
