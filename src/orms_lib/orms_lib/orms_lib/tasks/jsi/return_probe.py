from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.force_guarded_move import ForceGuardedMove
from geometry_msgs.msg import PoseStamped
import copy
import time

class ReturnProbeTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting ReturnProbe", 0.1)
        
        target_frame = "multimeter_probe_frame" # Use probe holder location
        target_pose = self.perception.wait_for_frame(target_frame)
        if not target_pose:
            self.node.get_logger().error(f"Failed to find {target_frame}")
            return False
            
        # 1. Approach socket holder from -5 cm X offset
        self.publish_feedback(goal_handle, "Approaching holder", 0.3)
        # self.robot.approach_in_object_frame(target_pose.transform, [-0.05, 0, 0], 2.0)
        
        # 2. Guarded touch along X-axis
        self.publish_feedback(goal_handle, "Guarded touch X", 0.5)
        guarded_move = ForceGuardedMove(self.robot)
        guarded_move.execute_x(force_threshold_x=5.0)
        
        # 3. Push probe into holder using Cartesian impedance
        #    Instead of apply_wrench (which is a no-op), we:
        #    - Drop X translational stiffness so the robot is compliant in push direction
        #    - Command a position 2cm beyond the expected insertion depth
        #    - The low stiffness limits the actual insertion force naturally
        self.publish_feedback(goal_handle, "Pushing probe into holder", 0.6)
        
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        STIFFNESS_PUSH_X  = [200.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_PUSH_X)
        
        # Slow down for the insertion
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.05, 0.05)
        
        # Push 2cm in +X to insert the probe (compliance limits force)
        self.robot.move_cartesian_relative([0.02, 0, 0], 2.0)
        
        # Wait for the probe to settle in the holder
        time.sleep(0.5)
        
        # Verify insertion depth -> check position change > 2mm
        
        # 4. Open gripper
        self.publish_feedback(goal_handle, "Releasing probe", 0.8)
        self.gripper.move(0.02)
        
        # 5. Retract upward 10 cm
        self.publish_feedback(goal_handle, "Retracting", 0.9)
        self.robot.move_cartesian_relative([0, 0, 0.1], 1.0)
        
        # Restore stiffness and speed
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
        self.robot.set_speed_scaling(old_vel, old_acc)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
