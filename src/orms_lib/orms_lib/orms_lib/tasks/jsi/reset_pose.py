from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.tasks.task_base import TaskBase

DEFAULT_START_JOINTS = [
    0.3338768184185028,
    -0.018447773531079292,
    0.16109512746334076,
    -1.7761679887771606,
    0.02443971298635006,
    1.7851251363754272,
    1.1781610250473022,
]


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
        self.start_joints = kwargs.get("start_joints", DEFAULT_START_JOINTS)
        self.duration = float(kwargs.get("duration", 5.0))

    def execute(self, goal_handle) -> bool:
        self.publish_feedback(goal_handle, "Starting ResetPose", 0.1)

        target_joints = list(self.start_joints)
        duration = self.duration

        request = goal_handle.request
        if hasattr(request, "param_keys"):
            if "start_joints" in request.param_keys:
                try:
                    idx = request.param_keys.index("start_joints")
                    val_str = request.param_values[idx]
                    target_joints = [float(x.strip()) for x in val_str.split(",") if x.strip()]
                except Exception as e:
                    self.node.get_logger().warn(
                        f"Failed to parse start_joints from goal parameter: {e}. Using default."
                    )
                    target_joints = self.start_joints

            if "duration" in request.param_keys:
                try:
                    idx = request.param_keys.index("duration")
                    duration = float(request.param_values[idx])
                except Exception as e:
                    self.node.get_logger().warn(
                        f"Failed to parse duration from goal parameter: {e}. Using default {duration}."
                    )

        self.node.get_logger().info("Sending robot to start pose...")
        self.publish_feedback(goal_handle, "Moving robot to start pose", 0.5)

        # Move joints using MoveIt planning
        success = self.robot.move_joints(target_joints, duration)

        if success:
            self.node.get_logger().info("Successfully reached start pose!")
            self.publish_feedback(goal_handle, "Complete", 1.0)
            return True
        else:
            self.node.get_logger().error("Failed to reach start pose.")
            self.publish_feedback(goal_handle, "Failed to reach start pose", 1.0)
            return False
