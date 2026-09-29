import time
from typing import Optional, List
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from orms_lib.interfaces.gripper_base import GripperBase


class MiaHandSkills:
    """
    Skill module for commanding specific poses on the Prensilia Mia Hand.
    """
    COMMAND_TOPIC = '/right_hand_controller/commands'

    # Joint order in right_hand_controller:
    # 0: right_hand_j_thumb_fle
    # 1: right_hand_j_index_fle
    # 2: right_hand_j_mrl_fle
    PRESSING_POSE: List[float] = [
        0.04999999888241291,  # right_hand_j_thumb_fle
        -0.9600000195205212,   # right_hand_j_index_fle
        0.019999999552965164, # right_hand_j_mrl_fle
    ]

    GRASP_PRESHAPE_POSE: List[float] = [
        0.15,  # right_hand_j_thumb_fle
        1.0,   # right_hand_j_index_fle
        1.2,   # right_hand_j_mrl_fle
    ]

    GRASP_POSE: List[float] = [
        0.35,  # right_hand_j_thumb_fle
        1.0,   # right_hand_j_index_fle
        1.2,   # right_hand_j_mrl_fle
    ]

    OPEN_HAND_POSE: List[float] = [
        0.0,  # right_hand_j_thumb_fle
        0.0,  # right_hand_j_index_fle
        0.0,  # right_hand_j_mrl_fle
    ]

    def __init__(self, gripper: Optional[GripperBase] = None, node: Optional[Node] = None):
        # Support passing (node, gripper) or (gripper, node) or just one of them
        if isinstance(gripper, Node) and not isinstance(node, Node):
            node, gripper = gripper, node

        self.gripper = gripper
        self.node = node if node is not None else getattr(gripper, 'node', None)

        self._cmd_pub = None
        if self.node is not None:
            self._cmd_pub = self.node.create_publisher(Float64MultiArray, self.COMMAND_TOPIC, 10)

    def set_pose(self, joint_positions: List[float], timeout_sec: float = 2.0) -> bool:
        """
        Send finger joint positions to right_hand_controller.
        Order: [thumb_fle, index_fle, mrl_fle].
        """
        if self.gripper is not None and hasattr(self.gripper, '_send_command'):
            self.gripper._send_command(joint_positions)
            if hasattr(self.gripper, '_wait_for_convergence'):
                return self.gripper._wait_for_convergence(joint_positions, timeout_sec=timeout_sec)
            time.sleep(min(timeout_sec, 1.0))
            return True

        if self._cmd_pub is not None:
            msg = Float64MultiArray()
            msg.data = [float(p) for p in joint_positions]
            self._cmd_pub.publish(msg)
            time.sleep(min(timeout_sec, 1.0))
            return True

        if self.node is not None:
            self.node.get_logger().error(
                "MiaHandSkills: Cannot send command, no publisher or gripper interface available."
            )
        return False

    def pressing_pose(self, timeout_sec: float = 2.0) -> bool:
        """
        Pose Mia Hand to form a pushing finger suitable for pressing buttons.
        Index finger flexed (~0.96), thumb extended (~0.05), middle/ring/little extended (~0.02).
        """
        if self.node is not None:
            self.node.get_logger().info("MiaHandSkills: Moving hand to pressing pose")
        return self.set_pose(self.PRESSING_POSE, timeout_sec=timeout_sec)

    def grasp_preshape(self, timeout_sec: float = 2.0) -> bool:
        """
        Pre-shape the Mia Hand for grasping: fingers partially open.
        """
        if self.node is not None:
            self.node.get_logger().info("MiaHandSkills: Moving hand to grasp preshape")
        return self.set_pose(self.GRASP_PRESHAPE_POSE, timeout_sec=timeout_sec)

    def grasp(self, timeout_sec: float = 2.0) -> bool:
        """
        Close the Mia Hand into grasp pose.
        """
        if self.node is not None:
            self.node.get_logger().info("MiaHandSkills: Grasping")
        return self.set_pose(self.GRASP_POSE, timeout_sec=timeout_sec)

    def open_hand(self, timeout_sec: float = 2.0) -> bool:
        """
        Fully open the Mia Hand (all joints to 0).
        """
        if self.node is not None:
            self.node.get_logger().info("MiaHandSkills: Opening hand")
        return self.set_pose(self.OPEN_HAND_POSE, timeout_sec=timeout_sec)
