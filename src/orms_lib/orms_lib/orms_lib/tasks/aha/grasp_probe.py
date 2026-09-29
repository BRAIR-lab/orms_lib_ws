import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.mia_hand_skills import MiaHandSkills


class GraspProbeTask(TaskBase):
    def __init__(self, node, robot: RobotBase, gripper: GripperBase, perception: PerceptionInterface, sim_mode: bool = False, **kwargs):
        super().__init__(node, robot, gripper, perception, sim_mode, **kwargs)
        self.target_frame = kwargs.get('target_frame', 'blue_button_frame')
        self.hand_skills = MiaHandSkills(gripper=self.gripper, node=self.node)

    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting GraspProbe", 0.1)

        # Read parameters
        request = goal_handle.request
        target_frame = self.target_frame
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        # 0. Preshape grasp
        self.hand_skills.grasp_preshape()
        self.publish_feedback(goal_handle, "Hand preshaped", 0.15)

        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        # Look up the target frame (blue_button_frame) in base
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
        # Translation: [0.318, 0.108, 0.064]
        # Rotation: [0.684, -0.169, -0.665, 0.247] (xyzw)
        # +2cm offset in Y, +1cm offset in X axis of blue_button_frame for pre-grasp clearance
        p_button_dorsum = np.array([0.323, 0.128, 0.064])
        r_button_dorsum = R.from_quat([0.684, -0.169, -0.665, 0.247])

        # 1. Move to approach pose (right_dorsum_link at target pose in blue_button_frame)
        self.publish_feedback(goal_handle, "Moving to approach pose", 0.3)
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

        # 2. Lower 4cm (move -4cm in blue_button_frame Y)
        self.publish_feedback(goal_handle, "Lowering 4cm", 0.5)
        p_button_lower = p_button_dorsum + np.array([0.0, -0.04, 0.0])
        p_base_lower = p_base_button + r_base_button.apply(p_button_lower)

        lower_pose = PoseStamped()
        lower_pose.header.frame_id = ref_frame
        lower_pose.pose.position.x = float(p_base_lower[0])
        lower_pose.pose.position.y = float(p_base_lower[1])
        lower_pose.pose.position.z = float(p_base_lower[2])
        lower_pose.pose.orientation.x = float(q_base_dorsum[0])
        lower_pose.pose.orientation.y = float(q_base_dorsum[1])
        lower_pose.pose.orientation.z = float(q_base_dorsum[2])
        lower_pose.pose.orientation.w = float(q_base_dorsum[3])

        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)

        self.robot.move_cartesian_path([lower_pose])

        self.robot.set_speed_scaling(old_vel, old_acc)

        # 3. Grasp
        self.publish_feedback(goal_handle, "Grasping", 0.65)
        self.hand_skills.grasp()

        # 4. Retract: pull probe 5cm in +Z of blue_button_frame
        self.publish_feedback(goal_handle, "Retracting", 0.8)
        p_button_retract = p_button_lower + np.array([0.0, 0.0, 0.05])
        p_base_retract = p_base_button + r_base_button.apply(p_button_retract)

        retract_pose = PoseStamped()
        retract_pose.header.frame_id = ref_frame
        retract_pose.pose.position.x = float(p_base_retract[0])
        retract_pose.pose.position.y = float(p_base_retract[1])
        retract_pose.pose.position.z = float(p_base_retract[2])
        retract_pose.pose.orientation.x = float(q_base_dorsum[0])
        retract_pose.pose.orientation.y = float(q_base_dorsum[1])
        retract_pose.pose.orientation.z = float(q_base_dorsum[2])
        retract_pose.pose.orientation.w = float(q_base_dorsum[3])

        self.robot.move_cartesian_path([retract_pose])

        # 5. Lift: go up 10cm (+10cm in blue_button_frame Y)
        self.publish_feedback(goal_handle, "Lifting", 0.9)
        p_button_lift = p_button_retract + np.array([0.0, 0.10, 0.0])
        p_base_lift = p_base_button + r_base_button.apply(p_button_lift)

        lift_pose = PoseStamped()
        lift_pose.header.frame_id = ref_frame
        lift_pose.pose.position.x = float(p_base_lift[0])
        lift_pose.pose.position.y = float(p_base_lift[1])
        lift_pose.pose.position.z = float(p_base_lift[2])
        lift_pose.pose.orientation.x = float(q_base_dorsum[0])
        lift_pose.pose.orientation.y = float(q_base_dorsum[1])
        lift_pose.pose.orientation.z = float(q_base_dorsum[2])
        lift_pose.pose.orientation.w = float(q_base_dorsum[3])

        self.robot.move_cartesian_path([lift_pose])

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
