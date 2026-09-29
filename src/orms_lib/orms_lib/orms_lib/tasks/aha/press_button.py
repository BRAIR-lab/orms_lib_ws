import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.mia_hand_skills import MiaHandSkills


class PressButtonTask(TaskBase):
    def __init__(self, node, robot: RobotBase, gripper: GripperBase, perception: PerceptionInterface, sim_mode: bool = False, **kwargs):
        super().__init__(node, robot, gripper, perception, sim_mode, **kwargs)
        self.target_frame = kwargs.get('target_frame', 'blue_button_frame')
        self.hand_skills = MiaHandSkills(gripper=self.gripper, node=self.node)

    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Approaching", 0.1)

        # Read parameters
        request = goal_handle.request
        target_frame = self.target_frame
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        # 1. Pose Mia Hand to form pushing finger
        self.hand_skills.pressing_pose()
        self.publish_feedback(goal_handle, "Hand Posed", 0.2)

        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        # 2. Get target frame (or use a fallback test pose)
        target_tf = self.perception.wait_for_frame(target_frame, source_frame=base_frame, timeout_sec=3.0)
        if not target_tf and base_frame != 'base':
            target_tf = self.perception.wait_for_frame(target_frame, source_frame='base', timeout_sec=1.0)

        if target_tf:
            p_base_button = np.array([
                target_tf.transform.translation.x,
                target_tf.transform.translation.y,
                target_tf.transform.translation.z
            ])
            r_base_button = R.from_quat([
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w
            ])
            ref_frame = target_tf.header.frame_id or base_frame
        else:
            self.node.get_logger().warn(
                f"Frame '{target_frame}' not found. Using fallback test pose."
            )
            p_base_button = np.array([0.4, 0.0, 0.4])
            r_base_button = R.identity()
            ref_frame = base_frame

        # Target pose of right_dorsum_link in blue_button_frame:
        # Translation: [0.102, 0.184, 0.022]
        # Rotation: [-0.626, 0.664, 0.134, -0.387] (xyzw)
        # +1cm offset in Z axis of blue_button_frame
        p_button_dorsum = np.array([0.102, 0.184, 0.032])
        r_button_dorsum = R.from_quat([-0.626, 0.664, 0.134, -0.387])

        # 3. Approach pose (right_dorsum_link at target pose in blue_button_frame)
        self.publish_feedback(goal_handle, "Moving to approach pose", 0.4)
        p_base_approach = p_base_button + r_base_button.apply(p_button_dorsum)
        r_base_dorsum = r_base_button * r_button_dorsum
        q_base_dorsum = r_base_dorsum.as_quat()

        approach_pose = PoseStamped()
        approach_pose.header.frame_id = ref_frame
        approach_pose.pose.position.x = float(p_base_approach[0])
        approach_pose.pose.position.y = float(p_base_approach[1])
        approach_pose.pose.position.z = float(p_base_approach[2])
        approach_pose.pose.orientation.x = float(q_base_dorsum[0])
        approach_pose.pose.orientation.y = float(q_base_dorsum[1])
        approach_pose.pose.orientation.z = float(q_base_dorsum[2])
        approach_pose.pose.orientation.w = float(q_base_dorsum[3])

        self.robot.move_cartesian(approach_pose, 5.0)

        # 4. Pure Cartesian downward push: go down 4cm (-4cm in blue_button_frame Y)
        self.publish_feedback(goal_handle, "Pushing button", 0.6)
        p_button_press = p_button_dorsum + np.array([0.0, -0.04, 0.0])
        p_base_press = p_base_button + r_base_button.apply(p_button_press)

        press_pose = PoseStamped()
        press_pose.header.frame_id = ref_frame
        press_pose.pose.position.x = float(p_base_press[0])
        press_pose.pose.position.y = float(p_base_press[1])
        press_pose.pose.position.z = float(p_base_press[2])
        press_pose.pose.orientation.x = float(q_base_dorsum[0])
        press_pose.pose.orientation.y = float(q_base_dorsum[1])
        press_pose.pose.orientation.z = float(q_base_dorsum[2])
        press_pose.pose.orientation.w = float(q_base_dorsum[3])

        # Slow down for contact
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)

        # Straight-line Cartesian path to press button
        self.robot.move_cartesian_path([press_pose])

        # Restore speed for retract
        self.robot.set_speed_scaling(old_vel, old_acc)

        # 5. Retract back to approach pose
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_path([approach_pose])

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
