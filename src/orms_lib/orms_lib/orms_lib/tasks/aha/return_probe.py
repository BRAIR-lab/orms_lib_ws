from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
import copy
import time


class ReturnProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting ReturnProbe", 0.1)
        
        target_frame = "multimeter_probe_frame"
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        
        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')
        
        if target_tf:
            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = target_tf.transform.translation.x
            target_pose.pose.position.y = target_tf.transform.translation.y
            target_pose.pose.position.z = target_tf.transform.translation.z
            target_pose.pose.orientation = target_tf.transform.rotation
        else:
            self.node.get_logger().warn(f"Failed to find {target_frame}. Using fallback.")
            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = 0.4
            target_pose.pose.position.y = 0.0
            target_pose.pose.position.z = 0.3
            target_pose.pose.orientation.w = 1.0
            
        # 1. Approach socket holder from -5 cm X offset
        self.publish_feedback(goal_handle, "Approaching holder", 0.3)
        approach_pose = copy.deepcopy(target_pose)
        approach_pose.pose.position.x -= 0.05
        self.robot.move_cartesian(approach_pose, 3.0)
        
        # 2. Push probe into holder via Cartesian relative move
        self.publish_feedback(goal_handle, "Pushing probe into holder", 0.6)
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.05, 0.05)
        
        # Push 2cm in +X to insert probe
        self.robot.move_cartesian_relative([0.02, 0, 0], 2.0)
        time.sleep(0.5)
        
        # 3. Open gripper
        self.publish_feedback(goal_handle, "Releasing probe", 0.8)
        self.gripper.move(0.02)
        
        # 4. Retract upward 10 cm
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_relative([0, 0, 0.1], 1.0)
        
        # Restore speed
        self.robot.set_speed_scaling(old_vel, old_acc)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
