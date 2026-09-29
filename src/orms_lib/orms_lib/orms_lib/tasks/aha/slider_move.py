from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
import copy


class SliderMoveNewTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Approaching", 0.1)

        request = goal_handle.request
        target_frame = "slider_frame"
        if "target_frame" in request.param_keys:
            idx = request.param_keys.index("target_frame")
            target_frame = request.param_values[idx]

        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        # 1. Pre-open the gripper
        self.publish_feedback(goal_handle, "Opening Gripper", 0.2)
        self.gripper.move(0.02)

        # 2. Get target frame
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        if target_tf:
            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = target_tf.transform.translation.x
            target_pose.pose.position.y = target_tf.transform.translation.y
            target_pose.pose.position.z = target_tf.transform.translation.z
            target_pose.pose.orientation = target_tf.transform.rotation
        else:
            self.node.get_logger().warn(
                f"Frame '{target_frame}' not found. Using fallback test pose."
            )
            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = 0.4
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.4
            target_pose.pose.orientation.x = 1.0
            target_pose.pose.orientation.y = 0.0
            target_pose.pose.orientation.z = 0.0
            target_pose.pose.orientation.w = 0.0

        # 3. Approach 5cm above target
        self.publish_feedback(goal_handle, "Moving overhead", 0.4)
        r_target = R.from_quat([
            target_pose.pose.orientation.x,
            target_pose.pose.orientation.y,
            target_pose.pose.orientation.z,
            target_pose.pose.orientation.w
        ])
        
        approach_offset = r_target.apply([0.0, 0.0, 0.05])
        
        approach_pose = PoseStamped()
        approach_pose.header = target_pose.header
        approach_pose.pose.position.x = target_pose.pose.position.x + approach_offset[0]
        approach_pose.pose.position.y = target_pose.pose.position.y + approach_offset[1]
        approach_pose.pose.position.z = target_pose.pose.position.z + approach_offset[2]
        
        # Rotate 180 around Y to point Z down and keep Y aligned
        r_rot_y = R.from_euler('y', 180, degrees=True)
        r_approach = r_target * r_rot_y
        quat = r_approach.as_quat()
        
        approach_pose.pose.orientation.x = quat[0]
        approach_pose.pose.orientation.y = quat[1]
        approach_pose.pose.orientation.z = quat[2]
        approach_pose.pose.orientation.w = quat[3]
        self.robot.move_cartesian(approach_pose, 5.0)

        # 4. Lower to grasp slider
        self.publish_feedback(goal_handle, "Lowering to contact", 0.5)
        contact_offset = r_target.apply([0.0, 0.0, -0.005])
        contact_pose = copy.deepcopy(approach_pose)
        contact_pose.pose.position.x = target_pose.pose.position.x + contact_offset[0]
        contact_pose.pose.position.y = target_pose.pose.position.y + contact_offset[1]
        contact_pose.pose.position.z = target_pose.pose.position.z + contact_offset[2]
        
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.1, 0.1)
        
        self.robot.move_cartesian_path([contact_pose])

        # Grasp slider peg
        self.publish_feedback(goal_handle, "Grasping slider", 0.55)
        self.gripper.close()

        # 5. Move slider positive Y
        self.publish_feedback(goal_handle, "Moving slider positive Y", 0.6)
        self.robot.set_speed_scaling(0.05, 0.05)

        slider_travel_dist = 0.034
        pos_y_vec = r_target.apply([0, slider_travel_dist, 0])
        
        slide_pose_end_1 = copy.deepcopy(contact_pose)
        slide_pose_end_1.pose.position.x += pos_y_vec[0]
        slide_pose_end_1.pose.position.y += pos_y_vec[1]
        slide_pose_end_1.pose.position.z += pos_y_vec[2]
        
        self.robot.move_cartesian_path([slide_pose_end_1])

        # 6. Move slider negative Y
        self.publish_feedback(goal_handle, "Moving slider negative Y", 0.7)
        neg_y_vec = r_target.apply([0, -slider_travel_dist, 0])
        
        slide_pose_end_2 = copy.deepcopy(slide_pose_end_1)
        slide_pose_end_2.pose.position.x += neg_y_vec[0]
        slide_pose_end_2.pose.position.y += neg_y_vec[1]
        slide_pose_end_2.pose.position.z += neg_y_vec[2]
        
        self.robot.move_cartesian_path([slide_pose_end_2])

        # 7. Release and retract
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.gripper.move(0.02)
        
        retract_pose = copy.deepcopy(slide_pose_end_2)
        retract_pose.pose.position.z = approach_pose.pose.position.z
        
        self.robot.move_cartesian_path([retract_pose])
        
        # Restore normal speed
        self.robot.set_speed_scaling(old_vel, old_acc)

        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
