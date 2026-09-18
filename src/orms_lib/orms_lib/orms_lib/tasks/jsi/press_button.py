from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.force_guarded_move import ForceGuardedMove
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R


class PressButtonTask(TaskBase):
    def __init__(self, node, robot: RobotBase, gripper: GripperBase, perception: PerceptionInterface, sim_mode: bool = False, **kwargs):
        super().__init__(node, robot, gripper, perception, sim_mode, **kwargs)
        self.target_frame = kwargs.get('target_frame', 'blue_button_frame')

    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Approaching", 0.1)

        # Read parameters
        request = goal_handle.request
        target_frame = self.target_frame
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        # 1. Close gripper to form pushing finger
        self.gripper.close()
        self.publish_feedback(goal_handle, "Gripper Closed", 0.2)

        # 2. Get target frame (or use a fallback test pose)
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        if target_tf:
            target_pose = PoseStamped()
            target_pose.header.frame_id = "fr3_link0"
            target_pose.pose.position.x = target_tf.transform.translation.x
            target_pose.pose.position.y = target_tf.transform.translation.y
            target_pose.pose.position.z = target_tf.transform.translation.z
            target_pose.pose.orientation = target_tf.transform.rotation
        else:
            self.node.get_logger().warn(
                f"Frame '{target_frame}' not found. Using fallback test pose."
            )
            # A reachable pose in front of the robot — good for testing
            target_pose = PoseStamped()
            target_pose.header.frame_id = "fr3_link0"
            target_pose.pose.position.x = 0.4
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.4
            target_pose.pose.orientation.x = 1.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.0

        # 3. Approach 5cm above target
        self.publish_feedback(goal_handle, "Moving overhead", 0.4)
        approach_pose = PoseStamped()
        approach_pose.header = target_pose.header
        approach_pose.pose.position.x = target_pose.pose.position.x
        approach_pose.pose.position.y = target_pose.pose.position.y
        approach_pose.pose.position.z = target_pose.pose.position.z + 0.05
        
        # Rotate target_pose.pose.orientation by 180 degrees around local Y axis
        r_target = R.from_quat([
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ])
        r_rot = R.from_euler('y', 180, degrees=True)
        r_approach = r_target * r_rot
        quat = r_approach.as_quat()
        
        approach_pose.pose.orientation.x = quat[0]
        approach_pose.pose.orientation.y = quat[1]
        approach_pose.pose.orientation.z = quat[2]
        approach_pose.pose.orientation.w = quat[3]
        self.robot.move_cartesian(approach_pose, 5.0)

        # 4. Pure stiffness downward push
        self.publish_feedback(goal_handle, "Pushing button (compliant)", 0.6)
        
        # Drop Z stiffness before pushing (using exact values from taskboard_manipulator)
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        STIFFNESS_PRESS_Z = [2000.0, 2000.0, 200.0,  200.0, 200.0, 200.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_PRESS_Z)
        
        # Exact target pose 5mm below the button surface
        press_pose = PoseStamped()
        press_pose.header = target_pose.header
        press_pose.pose.position.x = target_pose.pose.position.x
        press_pose.pose.position.y = target_pose.pose.position.y
        press_pose.pose.position.z = target_pose.pose.position.z - 0.00 # 0.005
        press_pose.pose.orientation = approach_pose.pose.orientation

        
        # 10% speed
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)
        
        # Strict straight-line Cartesian path
        self.robot.move_cartesian_path([press_pose])

        # Restore speed for retract
        self.robot.set_speed_scaling(old_vel, old_acc)

        # 5. Retract 5 cm upward (while still compliant to avoid sudden forces)
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_path([approach_pose])
        
        # Restore full stiffness after clearing the button
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
