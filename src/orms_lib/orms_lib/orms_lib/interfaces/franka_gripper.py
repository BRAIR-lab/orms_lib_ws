import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Grasp, Move
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup


from orms_lib.interfaces.gripper_base import GripperBase

class FrankaHandGripper(GripperBase):
    def __init__(self, node: Node):
        self.node = node
        self._cb_group = ReentrantCallbackGroup()

        # Try to connect to gripper action servers
        self.move_client = ActionClient(self.node, Move, '/franka_gripper/move', callback_group=self._cb_group)
        self.grasp_client = ActionClient(self.node, Grasp, '/franka_gripper/grasp', callback_group=self._cb_group)

        # Check availability with a short timeout
        self._gripper_available = self.move_client.wait_for_server(timeout_sec=3.0)
        if self._gripper_available:
            self.node.get_logger().info("Gripper action servers connected.")
        else:
            self.node.get_logger().warn(
                "Gripper action servers not available. "
                "Running in simulation mode — gripper commands will be no-ops."
            )

        # Gripper state
        self.width = 0.0
        self.state_sub = self.node.create_subscription(
            JointState,
            '/franka_gripper/joint_states',
            self._state_cb,
            10,
            callback_group=self._cb_group
        )

    def _state_cb(self, msg: JointState):
        # Assuming franka_gripper joint states publish the two fingers
        if len(msg.position) >= 2:
            self.width = msg.position[0] + msg.position[1]

    def get_width(self) -> float:
        return self.width

    def _wait_for_future(self, future, timeout_sec: float = 5.0) -> bool:
        start = time.time()
        while not future.done():
            if time.time() - start > timeout_sec:
                self.node.get_logger().error(f"Gripper action timed out after {timeout_sec}s")
                return False
            time.sleep(0.05)
        return True

    def move(self, width: float, speed: float = 0.1) -> bool:
        """
        Move gripper to a specific width.
        """
        self.node.get_logger().info(f"Moving gripper to width {width}m")
        if not self._gripper_available:
            self.node.get_logger().info("[SIM] Gripper move skipped — no hardware.")
            return True

        if not self.move_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("Gripper Move action server not available.")
            return False

        goal_msg = Move.Goal()
        goal_msg.width = width
        goal_msg.speed = speed

        future = self.move_client.send_goal_async(goal_msg)
        if not self._wait_for_future(future):
            return False

        result = future.result()
        if not result.accepted:
            return False

        get_result_future = result.get_result_async()
        if not self._wait_for_future(get_result_future, timeout_sec=10.0):
            return False

        return get_result_future.result().result.success

    def close(self, width: float = 0.0, speed: float = 0.1, force: float = 10.0,
              epsilon_inner: float = 0.005, epsilon_outer: float = 0.005) -> bool:
        """
        Grasp an object or close completely.
        """
        self.node.get_logger().info(f"Grasping (closing) gripper. Target width: {width}m")
        if not self._gripper_available:
            self.node.get_logger().info("[SIM] Gripper close skipped — no hardware.")
            return True

        if not self.grasp_client.wait_for_server(timeout_sec=2.0):
            self.node.get_logger().error("Gripper Grasp action server not available.")
            return False

        goal_msg = Grasp.Goal()
        goal_msg.width = width
        goal_msg.speed = speed
        goal_msg.force = force
        goal_msg.epsilon.inner = epsilon_inner
        goal_msg.epsilon.outer = epsilon_outer

        future = self.grasp_client.send_goal_async(goal_msg)
        if not self._wait_for_future(future):
            return False

        result = future.result()
        if not result.accepted:
            return False

        get_result_future = result.get_result_async()
        if not self._wait_for_future(get_result_future, timeout_sec=10.0):
            return False

        return get_result_future.result().result.success

    def open(self) -> bool:
        """
        Open the gripper to maximum width.
        """
        self.node.get_logger().info("Opening gripper.")
        # Franka hand maximum width is around 0.08m
        return self.move(0.08, speed=0.1)
