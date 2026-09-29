from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.mia_hand_skills import MiaHandSkills
from geometry_msgs.msg import PoseStamped


DEFAULT_RESET_POSE = [0.598, 0.114, 0.475, -0.014, 0.852, 0.070, 0.519]  # x, y, z, qx, qy, qz, qw


class ResetPoseTask(TaskBase):
    def __init__(
        self,
        node,
        robot: RobotBase,
        gripper: GripperBase,
        perception: PerceptionInterface,
        sim_mode: bool = False,
        **kwargs,
    ):
        super().__init__(node, robot, gripper, perception, sim_mode, **kwargs)
        self.reset_pose_coords = kwargs.get("reset_pose", DEFAULT_RESET_POSE)
        self.duration = float(kwargs.get("duration", 5.0))
        self.hand_skills = MiaHandSkills(gripper=self.gripper, node=self.node)

    def execute(self, goal_handle) -> bool:
        self.publish_feedback(goal_handle, "Starting ResetPose", 0.1)

        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        coords = list(self.reset_pose_coords)
        duration = self.duration

        request = goal_handle.request
        if hasattr(request, "param_keys"):
            if "reset_pose" in request.param_keys:
                try:
                    idx = request.param_keys.index("reset_pose")
                    val_str = request.param_values[idx]
                    coords = [float(x.strip()) for x in val_str.split(",") if x.strip()]
                except Exception as e:
                    self.node.get_logger().warn(
                        f"Failed to parse reset_pose parameter: {e}. Using default."
                    )
            elif "start_joints" in request.param_keys:
                self.node.get_logger().warn(
                    "start_joints provided but aha tasks use Cartesian reset pose. Using Cartesian default."
                )

            if "duration" in request.param_keys:
                try:
                    idx = request.param_keys.index("duration")
                    duration = float(request.param_values[idx])
                except Exception as e:
                    self.node.get_logger().warn(
                        f"Failed to parse duration: {e}. Using default {duration}."
                    )

        target = PoseStamped()
        target.header.frame_id = base_frame
        target.header.stamp = self.node.get_clock().now().to_msg()
        target.pose.position.x = coords[0]
        target.pose.position.y = coords[1]
        target.pose.position.z = coords[2]

        if len(coords) >= 7:
            target.pose.orientation.x = coords[3]
            target.pose.orientation.y = coords[4]
            target.pose.orientation.z = coords[5]
            target.pose.orientation.w = coords[6]
        else:
            target.pose.orientation.w = 1.0

        self.node.get_logger().info(
            f"Sending robot to Cartesian reset pose [{coords[0]}, {coords[1]}, {coords[2]}]..."
        )
        self.publish_feedback(goal_handle, "Moving robot to reset pose", 0.5)

        success = self.robot.move_cartesian(target, duration)

        if success:
            self.node.get_logger().info("Successfully reached reset pose!")
            self.hand_skills.open_hand()
            self.publish_feedback(goal_handle, "Complete", 1.0)
            return True
        else:
            self.node.get_logger().error("Failed to reach reset pose.")
            self.publish_feedback(goal_handle, "Failed to reach reset pose", 1.0)
            return False
