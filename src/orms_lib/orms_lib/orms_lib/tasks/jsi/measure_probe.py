from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.force_guarded_move import ForceGuardedMove

class MeasureProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting MeasureProbe", 0.1)
        
        target_frame = "multimeter_socket_frame"
        target_pose = self.perception.wait_for_frame(target_frame)
        if not target_pose:
            self.node.get_logger().error(f"Failed to find {target_frame}")
            return False
            
        # 1. Approach via point
        self.publish_feedback(goal_handle, "Moving to via point", 0.3)
        # self.robot.move_in_object_frame(via_pose, 2.0)
        
        # 2. Approach test point 2cm overhead
        self.publish_feedback(goal_handle, "Approaching test point", 0.5)
        # self.robot.approach_in_object_frame(target_pose.transform, [0, 0, 0.02], 2.0)
        
        # 3. Guarded downward touch (-3N)
        self.publish_feedback(goal_handle, "Touching test point", 0.7)
        guarded_move = ForceGuardedMove(self.robot)
        success = guarded_move.execute_z(force_threshold_z=-3.0)
        
        if not success:
            self.node.get_logger().error("Did not feel test point contact")
            return False
            
        # 4. Retract 5 cm
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        # self.robot.move_relative_in_object_frame([0, 0, 0.05], 1.0)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
