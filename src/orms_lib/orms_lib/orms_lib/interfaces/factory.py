from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
import rclpy

def create_robot(node: rclpy.node.Node, robot_type: str) -> RobotBase:
    if robot_type == "franka_fr3":
        from orms_lib.interfaces.franka_robot import FrankaRobot
        return FrankaRobot(node)
    elif robot_type == "ur5e":
        from orms_lib.interfaces.ur5e_robot import UR5eRobot
        return UR5eRobot(node)
    else:
        raise ValueError(f"Unknown robot_type in config: {robot_type}")

def create_gripper(node: rclpy.node.Node, gripper_type: str) -> GripperBase:
    if gripper_type == "franka_hand":
        from orms_lib.interfaces.franka_gripper import FrankaHandGripper
        return FrankaHandGripper(node)
    elif gripper_type == "robotiq_2f85":
        from orms_lib.interfaces.robotiq_gripper import RobotiqGripper
        return RobotiqGripper(node)
    else:
        raise ValueError(f"Unknown gripper_type in config: {gripper_type}")
