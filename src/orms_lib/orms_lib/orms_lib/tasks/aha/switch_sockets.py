from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped
import copy


class SwitchSocketsTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting SwitchSockets", 0.1)
        
        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        black_frame_name = "black_socket_frame"
        red_frame_name = "red_socket_frame"
        
        # 1. Approach and grasp plug at black socket
        self.publish_feedback(goal_handle, "Approaching black socket", 0.2)
        black_tf = self.perception.wait_for_frame(black_frame_name, timeout_sec=2.0)
        if black_tf:
            black_pose = PoseStamped()
            black_pose.header.frame_id = base_frame
            black_pose.pose.position.x = black_tf.transform.translation.x
            black_pose.pose.position.y = black_tf.transform.translation.y
            black_pose.pose.position.z = black_tf.transform.translation.z
            black_pose.pose.orientation = black_tf.transform.rotation
            
            # Approach 3cm above socket
            approach_black = copy.deepcopy(black_pose)
            approach_black.pose.position.z += 0.03
            self.robot.move_cartesian(approach_black, 3.0)
            
            # Lower to plug
            self.robot.move_cartesian(black_pose, 2.0)
        else:
            self.node.get_logger().warn(f"Frame {black_frame_name} not found. Using relative move.")
            self.robot.move_cartesian_relative([0, 0, -0.03], 2.0)
            
        self.gripper.close()
        
        # 2. Extract plug (lift 5cm)
        self.publish_feedback(goal_handle, "Extracting plug", 0.4)
        self.robot.move_cartesian_relative([0, 0, 0.05], 2.0)
        
        # 3. Transfer to red socket
        self.publish_feedback(goal_handle, "Moving to red socket", 0.6)
        red_tf = self.perception.wait_for_frame(red_frame_name, timeout_sec=2.0)
        if red_tf:
            red_pose = PoseStamped()
            red_pose.header.frame_id = base_frame
            red_pose.pose.position.x = red_tf.transform.translation.x
            red_pose.pose.position.y = red_tf.transform.translation.y
            red_pose.pose.position.z = red_tf.transform.translation.z
            red_pose.pose.orientation = red_tf.transform.rotation
            
            approach_red = copy.deepcopy(red_pose)
            approach_red.pose.position.z += 0.03
            self.robot.move_cartesian(approach_red, 3.0)
            
            # 4. Insert plug
            self.publish_feedback(goal_handle, "Inserting plug", 0.8)
            self.robot.move_cartesian(red_pose, 2.0)
        else:
            self.node.get_logger().warn(f"Frame {red_frame_name} not found. Using relative move.")
            self.robot.move_cartesian_relative([0.05, 0, 0], 2.0)
            self.robot.move_cartesian_relative([0, 0, -0.05], 2.0)
            
        # 5. Release and retract
        self.publish_feedback(goal_handle, "Releasing and retracting", 0.9)
        self.gripper.move(0.02)
        self.robot.move_cartesian_relative([0, 0, 0.1], 1.5)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
