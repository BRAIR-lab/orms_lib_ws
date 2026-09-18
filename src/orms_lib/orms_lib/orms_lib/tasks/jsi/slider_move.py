from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import copy

class SliderMoveNewTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Approaching", 0.1)

        request = goal_handle.request
        target_frame = "slider_frame"  # default frame
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        # 1. Pre-open the gripper
        self.publish_feedback(goal_handle, "Opening Gripper", 0.2)
        self.gripper.move(0.02)  # Open to 2cm to avoid hitting other components

        # 2. Get target frame
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        if target_tf:
            # The perception frame for the slider might be slightly off center from the peg
            # We apply a 1.5cm offset in the negative Y direction of the slider's local frame
            r_tf = R.from_quat([
                target_tf.transform.rotation.x,
                target_tf.transform.rotation.y,
                target_tf.transform.rotation.z,
                target_tf.transform.rotation.w
            ])
            # y_offset = r_tf.apply([0.0, -0.015, 0.0])
            # no offset for now, just use the target frame directly
            y_offset = r_tf.apply([0.0, 0.0, 0.0])
            
            target_pose = PoseStamped()
            target_pose.pose.position.x = target_tf.transform.translation.x + y_offset[0]
            target_pose.pose.position.y = target_tf.transform.translation.y + y_offset[1]
            target_pose.pose.position.z = target_tf.transform.translation.z + y_offset[2]
            target_pose.pose.orientation = target_tf.transform.rotation
        else:
            self.node.get_logger().warn(
                f"Frame '{target_frame}' not found. Using fallback test pose."
            )
            # A reachable pose in front of the robot — good for testing
            target_pose = PoseStamped()
            target_pose.pose.position.x = 0.4
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.4
            target_pose.pose.orientation.x = 1.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.0

        # 3. Approach 5cm above target (along target's local Z-axis)
        self.publish_feedback(goal_handle, "Moving overhead", 0.4)
        
        r_target = R.from_quat([
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ])
        
        # Calculate offset in global frame corresponding to 5cm in local Z
        approach_offset = r_target.apply([0.0, 0.0, 0.05])
        
        approach_pose = PoseStamped()
        approach_pose.pose.position.x = target_pose.pose.position.x + approach_offset[0]
        approach_pose.pose.position.y = target_pose.pose.position.y + approach_offset[1]
        approach_pose.pose.position.z = target_pose.pose.position.z + approach_offset[2]
        
        # Rotate target_pose.pose.orientation to align TCP correctly:
        # TCP Z-axis points down (negative Z of slider)
        # TCP Y-axis points to Y of slider
        # TCP X-axis points to negative X of slider
        
        # Rotate 180 around Y to point Z down and keep Y aligned
        r_rot_y = R.from_euler('y', 180, degrees=True)
        r_approach = r_target * r_rot_y
        
        quat = r_approach.as_quat()
        
        approach_pose.pose.orientation.x = quat[0]
        approach_pose.pose.orientation.y = quat[1]
        approach_pose.pose.orientation.z = quat[2]
        approach_pose.pose.orientation.w = quat[3]
        self.robot.move_cartesian(approach_pose, 5.0)

        # 4. Lower until it touches the taskboard/slider
        self.publish_feedback(goal_handle, "Lowering to contact", 0.5)
        
        # Drop Z stiffness before touching (similar to button press)
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        STIFFNESS_CONTACT_Z = [2000.0, 2000.0, 200.0,  200.0, 200.0, 200.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_CONTACT_Z)
        
        # Target pose 5mm below the slider surface to guarantee contact (along local Z)
        contact_offset = r_target.apply([0.0, 0.0, -0.005])
        contact_pose = copy.deepcopy(approach_pose)
        contact_pose.pose.position.x = target_pose.pose.position.x + contact_offset[0]
        contact_pose.pose.position.y = target_pose.pose.position.y + contact_offset[1]
        contact_pose.pose.position.z = target_pose.pose.position.z + contact_offset[2]
        
        # Slow down for contact and manipulation
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)
        
        # Strict straight-line downward path
        self.robot.move_cartesian_path([contact_pose])

        # Grasp the slider peg (Optional but assumed since we opened it earlier)
        self.publish_feedback(goal_handle, "Grasping slider", 0.55)
        self.gripper.close()

        # 5. Move slider positive Y
        self.publish_feedback(goal_handle, "Moving slider positive Y", 0.6)
        
        # To avoid breaking things and to sense force inherently, we drop X and Y stiffness
        # This makes the robot very compliant while sliding!
        STIFFNESS_SLIDE = [10.0, 10.0, 10.0,  10.0, 10.0, 10.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_SLIDE)
        
        # Slow down even more for the actual slide to prevent sudden force spikes
        self.robot.set_speed_scaling(0.05, 0.05)

        # Define travel distance for the slider
        slider_travel_dist = 0.034

        # Calculate vector for moving along the local positive Y axis of the target frame
        pos_y_vec = r_target.apply([0, slider_travel_dist, 0])
        
        slide_pose_end_1 = copy.deepcopy(contact_pose)
        slide_pose_end_1.pose.position.x += pos_y_vec[0]
        slide_pose_end_1.pose.position.y += pos_y_vec[1]
        slide_pose_end_1.pose.position.z += pos_y_vec[2]
        
        self.robot.move_cartesian_path([slide_pose_end_1])

        # 6. Move slider negative Y
        self.publish_feedback(goal_handle, "Moving slider negative Y", 0.7)
        
        # Move back to the other end. Since the total slider is 3cm long and we used high compliance,
        # we read the actual current physical pose to start the backward motion.
        current_tcp_pose = self.robot.get_ee_pose()
        if current_tcp_pose is None:
            self.node.get_logger().error("Failed to read current pose")
            return False
            
        neg_y_vec = r_target.apply([0, -slider_travel_dist, 0])
        
        slide_pose_end_2 = copy.deepcopy(current_tcp_pose)
        slide_pose_end_2.pose.position.x += neg_y_vec[0]
        slide_pose_end_2.pose.position.y += neg_y_vec[1]
        slide_pose_end_2.pose.position.z += neg_y_vec[2]
        
        self.robot.move_cartesian_path([slide_pose_end_2])

        # 7. Release and retract
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.gripper.move(0.02)  # Release the slider (keeping it small to avoid collision)
        
        retract_pose = copy.deepcopy(slide_pose_end_2)
        retract_pose.pose.position.z = approach_pose.pose.position.z
        
        self.robot.move_cartesian_path([retract_pose])
        
        # Restore normal stiffness and speed
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
        self.robot.set_speed_scaling(old_vel, old_acc)

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
