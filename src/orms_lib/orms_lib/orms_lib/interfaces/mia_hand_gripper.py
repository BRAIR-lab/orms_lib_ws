import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

from orms_lib.interfaces.gripper_base import GripperBase


class MiaHandGripper(GripperBase):
    """
    Hardware interface for the Prensilia Mia Hand, controlled via the
    right_hand_controller (position_controllers/JointGroupPositionController)
    from ros2_control.

    The JointGroupPositionController directly commands position interfaces
    for three finger DOFs:

        right_hand_controller:
            type: position_controllers/JointGroupPositionController
            joints:
              - right_hand_j_thumb_fle
              - right_hand_j_index_fle
              - right_hand_j_mrl_fle

    Finger joint conventions:
        0.0  = fully open (extended)
        ~1.0 = fully closed (flexed)   — exact limit depends on hardware

    The GripperBase API uses a scalar 'width' (in meters, 0.0 = closed,
    max = open).  This class maps width ↔ finger joint positions linearly.
    """

    # ── Finger DOF names (order must match the controller's joints list) ──
    DOF_NAMES = [
        'right_hand_j_thumb_fle',
        'right_hand_j_index_fle',
        'right_hand_j_mrl_fle',
    ]

    # ── Topic names ──
    COMMAND_TOPIC = '/right_hand_controller/commands'
    STATE_TOPIC = '/right_state_broadcaster/joint_states'

    # ── Finger joint limits (radians) ──
    JOINT_OPEN = 0.0      # fully extended
    JOINT_CLOSED = 1.0    # fully flexed (adjust to actual hardware limit)

    # ── Gripper width mapping ──
    # The "width" in the GripperBase API is an abstract scalar:
    #   MAX_WIDTH  ↔  all fingers at JOINT_OPEN
    #   0.0        ↔  all fingers at JOINT_CLOSED
    MAX_WIDTH = 0.10  # 10 cm effective aperture when fully open

    def __init__(self, node: Node):
        super().__init__(node)
        self._cb_group = ReentrantCallbackGroup()

        # ── Publisher: position commands ──
        self._cmd_pub = self.node.create_publisher(
            Float64MultiArray,
            self.COMMAND_TOPIC,
            10
        )

        # ── Subscriber: finger joint states ──
        self._finger_positions = {}  # joint_name → position
        self._state_sub = self.node.create_subscription(
            JointState,
            self.STATE_TOPIC,
            self._state_cb,
            qos_profile_sensor_data,
            callback_group=self._cb_group
        )

        self.node.get_logger().info(
            f"MiaHandGripper initialized. "
            f"Commands: {self.COMMAND_TOPIC}, "
            f"State: {self.STATE_TOPIC}"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _state_cb(self, msg: JointState):
        """Cache the latest finger joint positions."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.DOF_NAMES:
                self._finger_positions[name] = pos

    def _width_to_joint_positions(self, width: float) -> list:
        """
        Map a scalar gripper width (meters) to finger joint positions.
        width = MAX_WIDTH → all joints at JOINT_OPEN (extended)
        width = 0.0       → all joints at JOINT_CLOSED (flexed)
        """
        # Clamp width
        w = max(0.0, min(self.MAX_WIDTH, width))
        # Linear interpolation: width ∝ (1 - joint_fraction)
        fraction_open = w / self.MAX_WIDTH
        joint_pos = self.JOINT_OPEN + (1.0 - fraction_open) * (self.JOINT_CLOSED - self.JOINT_OPEN)
        # Same position for all three fingers
        return [joint_pos, joint_pos, joint_pos]

    def _joint_positions_to_width(self) -> float:
        """
        Estimate current gripper width from the mean finger flexion.
        """
        if not self._finger_positions:
            return 0.0

        positions = [
            self._finger_positions.get(name, self.JOINT_OPEN)
            for name in self.DOF_NAMES
        ]
        mean_pos = sum(positions) / len(positions)

        # Inverse of _width_to_joint_positions
        fraction_open = 1.0 - (mean_pos - self.JOINT_OPEN) / (self.JOINT_CLOSED - self.JOINT_OPEN)
        fraction_open = max(0.0, min(1.0, fraction_open))
        return fraction_open * self.MAX_WIDTH

    def _send_command(self, joint_positions: list) -> bool:
        """
        Publish a Float64MultiArray position command to the
        right_hand_controller (JointGroupPositionController).
        Order: [thumb_fle, index_fle, mrl_fle]
        """
        msg = Float64MultiArray()
        msg.data = [float(p) for p in joint_positions]
        self._cmd_pub.publish(msg)
        return True

    def _wait_for_convergence(self, target_positions: list,
                               tolerance: float = 0.05,
                               timeout_sec: float = 5.0) -> bool:
        """
        Wait until finger joints converge to target positions.
        """
        start = time.time()
        while time.time() - start < timeout_sec:
            if self._finger_positions:
                errors = []
                for name, target in zip(self.DOF_NAMES, target_positions):
                    current = self._finger_positions.get(name, None)
                    if current is not None:
                        errors.append(abs(current - target))
                if errors and max(errors) < tolerance:
                    return True
            time.sleep(0.05)
        self.node.get_logger().warn(
            f"Gripper convergence timed out after {timeout_sec}s"
        )
        return False

    # ------------------------------------------------------------------
    # Public API — GripperBase implementation
    # ------------------------------------------------------------------

    def move(self, width: float, speed: float = 0.1) -> bool:
        """
        Move the Mia hand fingers to achieve the specified aperture width.
        The 'speed' parameter is accepted for API compatibility but the
        actual tracking speed is governed by the position controller.
        """
        self.node.get_logger().info(
            f"Moving Mia hand to width {width:.3f}m "
            f"(max {self.MAX_WIDTH:.3f}m)"
        )
        joint_positions = self._width_to_joint_positions(width)
        self._send_command(joint_positions)

        # Wait for fingers to reach target
        return self._wait_for_convergence(joint_positions, timeout_sec=5.0)

    def close(self, width: float = 0.0, speed: float = 0.1,
              force: float = 10.0, epsilon_inner: float = 0.005,
              epsilon_outer: float = 0.005) -> bool:
        """
        Close the Mia hand (flex all fingers).
        The force/epsilon parameters are accepted for API compatibility
        but are not used — the position controller tracks a command directly.
        """
        self.node.get_logger().info(
            f"Closing Mia hand (target width: {width:.3f}m)"
        )
        joint_positions = self._width_to_joint_positions(width)
        self._send_command(joint_positions)
        return self._wait_for_convergence(joint_positions, timeout_sec=5.0)

    def open(self) -> bool:
        """
        Open the Mia hand fully (extend all fingers).
        """
        self.node.get_logger().info("Opening Mia hand.")
        joint_positions = [self.JOINT_OPEN] * 3
        self._send_command(joint_positions)
        return self._wait_for_convergence(joint_positions, timeout_sec=5.0)

    def get_width(self) -> float:
        """
        Returns the current estimated gripper width (meters).
        Computed from the mean finger flexion angle.
        """
        return self._joint_positions_to_width()
