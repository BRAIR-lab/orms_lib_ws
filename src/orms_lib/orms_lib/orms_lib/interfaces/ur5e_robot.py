import rclpy
import time
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from typing import List
import tf2_ros

from orms_lib.interfaces.robot_base import RobotBase


class UR5eRobot(RobotBase):
    """
    Hardware abstraction layer for UR5e via the CartesianMotionController
    from the cartesian_controllers package (ros2_control).

    Motion is purely Cartesian: the controller streams toward a target_frame
    topic and publishes current_pose feedback.  No MoveIt / joint-space
    planning is used.

    Controller reference (from bimanual_hand_controllers.yaml):
        right_cartesian_controller:
            type: cartesian_motion_controller/CartesianMotionController
            end_effector_link: right_dorsum_link
            robot_base_link: world
            joints: [right_shoulder_pan_joint .. right_wrist_3_joint]
            command_interfaces: [position]
            solver.publish_state_feedback: True
    """

    # Frame names — match the controller config
    BASE_FRAME = 'world'
    EE_LINK = 'right_dorsum_link'

    # Joint names for the right UR5e arm
    JOINT_NAMES = [
        'right_shoulder_pan_joint',
        'right_shoulder_lift_joint',
        'right_elbow_joint',
        'right_wrist_1_joint',
        'right_wrist_2_joint',
        'right_wrist_3_joint',
    ]

    # Controller topic names
    TARGET_FRAME_TOPIC = '/right_cartesian_controller/target_frame'
    CURRENT_POSE_TOPIC = '/right_cartesian_controller/current_pose'

    # Convergence thresholds
    POSITION_TOLERANCE = 0.005    # 5 mm
    ORIENTATION_TOLERANCE = 0.02  # ~1.1 degrees in quaternion distance

    def __init__(self, node: Node):
        super().__init__(node)
        self._cb_group = ReentrantCallbackGroup()

        # Velocity / acceleration scaling (used for duration estimation)
        self.velocity_scaling = 0.1
        self.acceleration_scaling = 0.1

        # ── Publisher: Cartesian target ──
        self._target_pub = self.node.create_publisher(
            PoseStamped,
            self.TARGET_FRAME_TOPIC,
            10
        )

        # ── Subscriber: Cartesian current pose feedback ──
        self._current_pose = None
        self._current_pose_sub = self.node.create_subscription(
            PoseStamped,
            self.CURRENT_POSE_TOPIC,
            self._current_pose_cb,
            qos_profile_sensor_data,
            callback_group=self._cb_group
        )

        # ── TF for get_ee_pose (fallback / additional lookups) ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

        # ── Wrench — not bridged for this setup, return zeros ──
        self.current_wrench = [0.0] * 6

        self.node.get_logger().info(
            "UR5eRobot interface initialized. "
            f"Cartesian controller: {self.TARGET_FRAME_TOPIC}"
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _current_pose_cb(self, msg: PoseStamped):
        """Cache the latest EE pose from the Cartesian controller."""
        self._current_pose = msg

    @staticmethod
    def _position_distance(p1, p2) -> float:
        """Euclidean distance between two geometry_msgs/Point."""
        return math.sqrt(
            (p1.x - p2.x) ** 2 +
            (p1.y - p2.y) ** 2 +
            (p1.z - p2.z) ** 2
        )

    @staticmethod
    def _quaternion_distance(q1, q2) -> float:
        """
        Angular distance between two quaternions.
        Returns a value in [0, 1] where 0 = identical, 1 = 180° apart.
        Uses 1 - |dot(q1, q2)| which is fast and monotonic.
        """
        dot = abs(
            q1.x * q2.x +
            q1.y * q2.y +
            q1.z * q2.z +
            q1.w * q2.w
        )
        return 1.0 - min(dot, 1.0)

    def _wait_for_convergence(self, target_pose: PoseStamped,
                               timeout_sec: float,
                               stop_condition=None) -> bool:
        """
        Block until the current EE pose converges to the target,
        a stop_condition fires, or the timeout expires.
        Returns True on convergence or stop_condition, False on timeout.
        """
        start = time.time()
        rate_hz = 50
        sleep_time = 1.0 / rate_hz

        while True:
            elapsed = time.time() - start
            if elapsed > timeout_sec:
                self.node.get_logger().error(
                    f"Cartesian move timed out after {timeout_sec:.1f}s"
                )
                return False

            if stop_condition is not None and stop_condition():
                self.node.get_logger().info("Stop condition met. Motion halted.")
                # Publish current pose as target to freeze in place
                if self._current_pose is not None:
                    self._target_pub.publish(self._current_pose)
                return True

            if self._current_pose is not None:
                pos_err = self._position_distance(
                    self._current_pose.pose.position,
                    target_pose.pose.position
                )
                ori_err = self._quaternion_distance(
                    self._current_pose.pose.orientation,
                    target_pose.pose.orientation
                )
                if (pos_err < self.POSITION_TOLERANCE and
                        ori_err < self.ORIENTATION_TOLERANCE):
                    return True

            time.sleep(sleep_time)

    # ------------------------------------------------------------------
    # Public API — RobotBase implementation
    # ------------------------------------------------------------------

    def set_speed_scaling(self, velocity: float, acceleration: float):
        """
        Store speed scaling factors.  The CartesianMotionController's
        pd_gains govern actual tracking speed; these values are used
        for timeout estimation.
        """
        self.velocity_scaling = max(0.01, min(1.0, velocity))
        self.acceleration_scaling = max(0.01, min(1.0, acceleration))
        self.node.get_logger().info(
            f"Speed scaling updated: vel={self.velocity_scaling*100:.0f}%, "
            f"accel={self.acceleration_scaling*100:.0f}%"
        )

    def move_cartesian(self, target_pose: PoseStamped,
                       duration: float = 5.0,
                       stop_condition=None) -> bool:
        """
        Move end-effector to an absolute Cartesian pose by publishing
        to the CartesianMotionController's target_frame topic, then
        waiting for convergence.
        """
        self.node.get_logger().info(
            f"Moving Cartesian to "
            f"[{target_pose.pose.position.x:.3f}, "
            f"{target_pose.pose.position.y:.3f}, "
            f"{target_pose.pose.position.z:.3f}] "
            f"(timeout {duration:.1f}s)"
        )

        # Ensure the header is populated
        if not target_pose.header.frame_id:
            target_pose.header.frame_id = self.BASE_FRAME
        target_pose.header.stamp = self.node.get_clock().now().to_msg()

        # Publish target
        self._target_pub.publish(target_pose)

        # Wait for convergence (allow up to 3x duration for safety)
        timeout = max(duration * 3, 10.0)
        success = self._wait_for_convergence(
            target_pose, timeout, stop_condition
        )

        if success:
            self.node.get_logger().info("Cartesian move converged.")
        return success

    def move_cartesian_path(self, waypoints: List[PoseStamped]) -> bool:
        """
        Execute a sequence of Cartesian waypoints by streaming each
        target to the controller and waiting for convergence before
        advancing to the next waypoint.
        """
        self.node.get_logger().info(
            f"Executing Cartesian path with {len(waypoints)} waypoints."
        )
        for i, wp in enumerate(waypoints):
            self.node.get_logger().info(f"  Waypoint {i+1}/{len(waypoints)}")
            if not self.move_cartesian(wp, duration=5.0):
                self.node.get_logger().error(
                    f"Failed at waypoint {i+1}/{len(waypoints)}"
                )
                return False
        return True

    def move_cartesian_relative(self, delta_pos: List[float],
                                 duration: float = 3.0,
                                 stop_condition=None) -> bool:
        """
        Move end-effector by a relative Cartesian offset [dx, dy, dz],
        preserving current orientation.
        """
        self.node.get_logger().info(
            f"Relative Cartesian move {delta_pos} over {duration}s"
        )
        current = self.get_ee_pose()
        if current is None:
            self.node.get_logger().error(
                "Cannot get current pose for relative move."
            )
            return False

        target = PoseStamped()
        target.header = current.header
        target.pose.position.x = current.pose.position.x + delta_pos[0]
        target.pose.position.y = current.pose.position.y + delta_pos[1]
        target.pose.position.z = current.pose.position.z + delta_pos[2]
        target.pose.orientation = current.pose.orientation
        return self.move_cartesian(target, duration, stop_condition=stop_condition)

    def move_joints(self, joint_positions: List[float],
                    duration: float = 5.0) -> bool:
        """
        Joint-space motion is not used in this configuration.
        All motion is executed via the CartesianMotionController.
        """
        self.node.get_logger().warn(
            "UR5eRobot.move_joints called but joint-space controller "
            "is not bridged. Use move_cartesian instead."
        )
        return False

    def get_ee_pose(self) -> PoseStamped:
        """
        Returns the current end-effector pose.
        Primary source: cached pose from /right_cartesian_controller/current_pose.
        Fallback: TF lookup (world → right_dorsum_link).
        """
        # Try the controller's streamed pose first (lower latency)
        if self._current_pose is not None:
            return self._current_pose

        # Fallback to TF
        try:
            transform = self._tf_buffer.lookup_transform(
                self.BASE_FRAME,
                self.EE_LINK,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=3.0)
            )
            pose_msg = PoseStamped()
            pose_msg.header = transform.header
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            pose_msg.pose.orientation = transform.transform.rotation
            return pose_msg
        except tf2_ros.TransformException as ex:
            self.node.get_logger().warn(f"Could not get current EE pose: {ex}")
            return None

    def get_wrench(self) -> List[float]:
        """
        Returns current F/T readings [fx, fy, fz, tx, ty, tz].
        Force/torque sensor is not bridged in this configuration;
        returns zeros.
        """
        return list(self.current_wrench)
