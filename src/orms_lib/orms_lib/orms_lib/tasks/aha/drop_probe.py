from orms_lib.tasks.task_base import TaskBase
from geometry_msgs.msg import PoseStamped


class DropProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting DropProbe", 0.1)
        
        base_frame = getattr(self.robot, 'BASE_FRAME', 'world')

        # 1. Move to drop pose
        drop_pose = PoseStamped()
        drop_pose.header.frame_id = base_frame
        drop_pose.pose.position.x = 0.45
        drop_pose.pose.position.y = -0.15
        drop_pose.pose.position.z = 0.2
        drop_pose.pose.orientation.w = 1.0
        
        self.publish_feedback(goal_handle, "Moving to drop location", 0.3)
        self.robot.move_cartesian(drop_pose, 3.0)
        
        # 2. Lower arm 15 cm
        self.publish_feedback(goal_handle, "Lowering arm", 0.5)
        self.robot.move_cartesian_relative([0, 0, -0.15], 2.0)
        
        # 3. Open gripper
        self.publish_feedback(goal_handle, "Opening gripper", 0.7)
        self.gripper.open()
        
        # 4. Retract arm upward 15 cm
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_relative([0, 0, 0.15], 2.0)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
