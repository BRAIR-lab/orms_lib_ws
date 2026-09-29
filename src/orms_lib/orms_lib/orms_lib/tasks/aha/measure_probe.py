from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
import copy
import time


class MeasureProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting MeasureProbe", 0.1)
        
        target_frame = "multimeter_socket_frame"
        target_tf = self.perception.wait_for_frame(target_frame, timeout_sec=3.0)
        
        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        if target_tf:
            target_pose = PoseStamped()
            target_pose.header.frame_id = base_frame
            target_pose.pose.position.x = target_tf.transform.translation.x
            target_pose.pose.position.y = target_tf.transform.translation.y
            target_pose.pose.position.z = target_tf.transform.translation.z
            target_pose.pose.orientation = target_tf.transform.rotation
            
            # 1. Approach test point 2cm overhead
            self.publish_feedback(goal_handle, "Approaching test point", 0.4)
            approach_pose = copy.deepcopy(target_pose)
            approach_pose.pose.position.z += 0.02
            self.robot.move_cartesian(approach_pose, 3.0)
            
            # 2. Lower to touch test point
            self.publish_feedback(goal_handle, "Touching test point", 0.7)
            self.robot.move_cartesian(target_pose, 2.0)
        else:
            self.node.get_logger().warn(f"Failed to find {target_frame}. Using relative moves.")
            self.robot.move_cartesian_relative([0, 0, -0.02], 2.0)
            
        time.sleep(1.0)
        
        # 3. Retract 5 cm
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_relative([0, 0, 0.05], 1.5)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
