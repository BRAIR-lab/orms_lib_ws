from orms_lib.tasks.task_base import TaskBase
import time


class WrapCableTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting WrapCable", 0.1)
        
        # 1. Grab cable plug at red socket, lightly pinch
        self.publish_feedback(goal_handle, "Grabbing cable", 0.2)
        self.gripper.move(0.005)
        
        self.publish_feedback(goal_handle, "Sliding along cable", 0.3)
        self.gripper.move(0.01)
        self.robot.move_cartesian_relative([-0.10, 0, 0], 2.0)
        self.gripper.close()
        
        # 2. Execute wrapping trajectory
        self.publish_feedback(goal_handle, "Executing wrapping motion", 0.6)
        # Cartesian loop/arc around the holders
        self.robot.move_cartesian_relative([0, 0.05, 0.05], 2.0)
        self.robot.move_cartesian_relative([0.10, 0, 0], 2.0)
        self.robot.move_cartesian_relative([0, -0.05, -0.05], 2.0)
        
        # 3. Retract
        self.publish_feedback(goal_handle, "Releasing cable", 0.9)
        self.gripper.open()
        self.robot.move_cartesian_relative([0, 0, 0.05], 1.5)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
