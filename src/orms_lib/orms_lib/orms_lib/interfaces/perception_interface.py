import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Trigger
import time

class PerceptionInterface:
    def __init__(self, node: Node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        
        # Subscriptions
        self.slider_value = 0.0
        self.slider_sub = self.node.create_subscription(
            Float32,
            '/tum_tb_perception2/slider_solver_result',
            self._slider_cb,
            10
        )
        
        # Service Clients
        self.trigger_client = self.node.create_client(
            Trigger,
            '/tum_tb_perception2/rerun_pose_estimation'
        )
        self.slider_trigger_client = self.node.create_client(
            Trigger,
            '/tum_tb_perception2/slider_solver_trigger_srv'
        )

    def _slider_cb(self, msg: Float32):
        self.slider_value = msg.data

    def get_slider_value(self) -> float:
        return self.slider_value

    def trigger_detection(self):
        req = Trigger.Request()
        if self.trigger_client.wait_for_service(timeout_sec=1.0):
            self.trigger_client.call_async(req)
            self.node.get_logger().info("Triggered perception detection (async).")
        else:
            self.node.get_logger().warn("Perception trigger service not available.")

    def trigger_slider_solver(self):
        """Trigger the slider task solver to estimate the slider motion distance."""
        req = Trigger.Request()
        if self.slider_trigger_client.wait_for_service(timeout_sec=1.0):
            self.slider_trigger_client.call_async(req)
            self.node.get_logger().info("Triggered slider solver (async).")
        else:
            self.node.get_logger().warn("Slider trigger service not available.")

    def wait_for_frame(self, target_frame: str, source_frame: str = 'base', timeout_sec: float = 5.0) -> TransformStamped:
        """
        Wait for a TF frame to become available and return the transform.
        """
        try:
            # Wait for the transform
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout_sec)
            )
            return transform
        except tf2_ros.TransformException as ex:
            self.node.get_logger().error(f"Could not transform {source_frame} to {target_frame}: {ex}")
            return None

    def get_taskboard_frame(self, base_frame: str = 'base') -> TransformStamped:
        return self.wait_for_frame('taskboard_frame', base_frame)

    def get_component_frame(self, label: str, base_frame: str = 'base') -> TransformStamped:
        frame_id = f"{label}_frame"
        return self.wait_for_frame(frame_id, base_frame)
