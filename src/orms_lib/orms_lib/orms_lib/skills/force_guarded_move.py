import rclpy
import time
from geometry_msgs.msg import Pose
from orms_lib.interfaces.robot_base import RobotBase


class ForceGuardedMove:
    def __init__(self, robot: RobotBase, sim_mode: bool = True):
        self.robot = robot
        self.sim_mode = sim_mode

    def execute_z(self, force_threshold_z: float, max_distance: float = 0.1, speed: float = 0.01) -> bool:
        """
        Equivalent to TouchObjZPlane.
        Moves down along Z axis until force threshold is exceeded.
        In sim_mode, performs the downward motion and returns success.
        """
        self.robot.node.get_logger().info(
            f"Executing force guarded move along Z. Threshold: {force_threshold_z}N"
        )

        if self.sim_mode:
            # In simulation, just do the downward motion and report success
            self.robot.node.get_logger().info(
                f"[SIM] Moving down {max_distance}m (force sensing disabled)"
            )
            self.robot.move_cartesian_relative([0, 0, -max_distance], 3.0)
            return True

        # --- Real hardware path ---
        force_hit = [False]
        
        # Tare the sensor by saving the initial resting force (payload weight)
        import time
        time.sleep(0.5) # Wait for robot to settle
        initial_wrench = self.robot.get_wrench()
        fz_initial = initial_wrench[2]
        
        def check_force():
            current_wrench = self.robot.get_wrench()
            fz = current_wrench[2]
            delta_fz = fz - fz_initial
            if abs(delta_fz) > abs(force_threshold_z):
                force_hit[0] = True
                return True
            return False

        duration = max_distance / speed
        self.robot.node.get_logger().info("Starting force guarded downward motion (compliant)...")
        
        # Drop speed to 2% for the approach
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.02, 0.02)
        
        self.robot.move_cartesian_relative([0, 0, -max_distance], duration, stop_condition=check_force)
        
        # Restore speed
        self.robot.set_speed_scaling(old_vel, old_acc)

        if force_hit[0]:
            self.robot.node.get_logger().info("Force threshold reached. Stopping.")
            return True

        self.robot.node.get_logger().warning("Force guarded move reached max distance without hitting threshold.")
        return False

    def execute_x(self, force_threshold_x: float, max_distance: float = 0.1, speed: float = 0.01) -> bool:
        """
        Equivalent to TouchObjXPlane.
        """
        self.robot.node.get_logger().info(
            f"Executing force guarded move along X. Threshold: {force_threshold_x}N"
        )

        if self.sim_mode:
            self.robot.node.get_logger().info(
                f"[SIM] Moving along X {max_distance}m (force sensing disabled)"
            )
            self.robot.move_cartesian_relative([max_distance, 0, 0], 3.0)
            return True

        force_hit = [False]
        
        import time
        time.sleep(0.5)
        initial_wrench = self.robot.get_wrench()
        fx_initial = initial_wrench[0]
        
        def check_force_x():
            current_wrench = self.robot.get_wrench()
            fx = current_wrench[0]
            delta_fx = fx - fx_initial
            if abs(delta_fx) > abs(force_threshold_x):
                force_hit[0] = True
                return True
            return False

        duration = max_distance / speed
        self.robot.node.get_logger().info("Starting force guarded motion along X (compliant)...")
        
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling
        self.robot.set_speed_scaling(0.02, 0.02)
        
        self.robot.move_cartesian_relative([max_distance, 0, 0], duration, stop_condition=check_force_x)
        
        self.robot.set_speed_scaling(old_vel, old_acc)

        if force_hit[0]:
            self.robot.node.get_logger().info("Force threshold reached. Stopping.")
            return True

        self.robot.node.get_logger().warning("Force guarded move reached max distance without hitting threshold.")
        return False
