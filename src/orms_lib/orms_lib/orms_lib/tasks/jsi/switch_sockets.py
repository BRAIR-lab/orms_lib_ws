from orms_lib.tasks.task_base import TaskBase
from orms_lib.skills.force_guarded_move import ForceGuardedMove
from orms_lib.skills.spiral_search import SpiralSearch
import copy

class SwitchSocketsTask(TaskBase):
    def execute(self, goal_handle):
        self.publish_feedback(goal_handle, "Starting SwitchSockets", 0.1)
        
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        
        # We need two frames. In perception, both might be "multimeter_connector_frame"
        # We assume the caller passed the correct target frames or coordinates in params
        black_frame_name = "black_socket_frame" # Placeholder
        red_frame_name = "red_socket_frame"     # Placeholder
        
        # 1. Unplugging
        self.publish_feedback(goal_handle, "Approaching black socket", 0.2)
        # self.robot.approach_in_object_frame(black_pose, [0, 0, 0.02], 2.0)
        
        self.publish_feedback(goal_handle, "Touching plug", 0.3)
        guarded_move = ForceGuardedMove(self.robot)
        guarded_move.execute_z(force_threshold_z=-2.0)
        
        # self.robot.move_relative_in_object_frame([0, 0, 0.007], 0.5)
        self.gripper.close()
        
        self.publish_feedback(goal_handle, "Extracting plug", 0.4)
        # self.robot.move_relative_in_object_frame([0, 0, 0.026], 1.0)
        
        # 2. Transfer
        self.publish_feedback(goal_handle, "Moving to red socket", 0.5)
        # self.robot.approach_in_object_frame(red_pose, [0, 0, 0.03], 1.0)
        
        # Touch down to establish plate height
        guarded_move.execute_z(force_threshold_z=-5.0)
        current_pose = self.robot.get_ee_pose()
        if current_pose is None:
            self.node.get_logger().error("Failed to read current pose")
            return False
        h_plate = current_pose.pose.position.z
        
        # 3. Spiral Search (now uses Cartesian impedance internally)
        self.publish_feedback(goal_handle, "Spiral search", 0.7)
        spiral = SpiralSearch(self.robot)
        found = spiral.execute(z_plate=h_plate)
        if not found:
            self.node.get_logger().error("Failed to find red socket hole")
            return False
            
        # 4. Insertion — use Cartesian impedance instead of apply_wrench
        #    Drop Z stiffness and command a position below the insertion point.
        #    The impedance controller limits the actual insertion force.
        self.publish_feedback(goal_handle, "Inserting plug", 0.8)
        
        STIFFNESS_INSERT_Z = [2000.0, 2000.0, 200.0, 200.0, 200.0, 200.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_INSERT_Z)
        
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.05, 0.05)
        
        # Push 2cm downward to insert the plug (compliance limits force)
        self.robot.move_cartesian_relative([0, 0, -0.02], 2.0)
        # Add lateral wiggle if jammed (omitted for brevity)
        
        # 5. Release
        self.publish_feedback(goal_handle, "Releasing and retracting", 0.9)
        self.gripper.move(0.02)
        
        # Retract upward
        self.robot.move_cartesian_relative([0, 0, 0.1], 1.0)
        
        # Restore stiffness and speed
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
        self.robot.set_speed_scaling(old_vel, old_acc)
        
        self.publish_feedback(goal_handle, "Complete", 1.0)
        return True
