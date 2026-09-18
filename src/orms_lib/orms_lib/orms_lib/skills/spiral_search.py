import time
import math
import copy
from geometry_msgs.msg import PoseStamped
from orms_lib.interfaces.robot_base import RobotBase

class SpiralSearch:
    def __init__(self, robot: RobotBase):
        self.robot = robot

    def execute(self, z_plate: float, max_radius: float = 0.02, push_force: float = -3.0) -> bool:
        """
        Archimedean spiral search using Cartesian impedance control.
        Equivalent to SpiralSearch in SwitchSockets3.m

        Instead of apply_wrench() (which is a no-op in MoveIt mode), we use
        set_cartesian_stiffness() with low Z stiffness and command positions
        slightly below the plate surface. The impedance controller naturally
        produces a compliant downward push.
        """
        self.robot.node.get_logger().info("Executing spiral search (Cartesian impedance)")

        # Save current speed for restoration
        old_vel = self.robot.velocity_scaling
        old_acc = self.robot.acceleration_scaling

        # Enable compliance: low Z stiffness for downward push,
        # moderate XY for spiral motion, low rotational for compliance
        STIFFNESS_DEFAULT = [2000.0, 2000.0, 2000.0, 200.0, 200.0, 200.0]
        STIFFNESS_SPIRAL  = [2000.0, 2000.0, 100.0, 30.0, 30.0, 30.0]
        self.robot.set_cartesian_stiffness(STIFFNESS_SPIRAL)
        self.robot.set_speed_scaling(0.03, 0.03)

        # Get current EE pose as the starting point for the spiral
        start_pose = self.robot.get_ee_pose()
        if start_pose is None:
            self.robot.node.get_logger().error("Failed to read current pose for spiral search")
            self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
            self.robot.set_speed_scaling(old_vel, old_acc)
            return False

        # Build spiral waypoints
        # We command Z slightly below the plate to produce a gentle push via compliance.
        # If the probe finds a hole, Z will drop further.
        z_command = z_plate - 0.003  # 3mm below plate surface

        r = 0.0
        theta = 0.0
        dr = 0.0005   # radius increment per step
        dtheta = 0.3   # angle increment per step

        waypoints = []
        while r < max_radius:
            x_offset = r * math.cos(theta)
            y_offset = r * math.sin(theta)

            wp = copy.deepcopy(start_pose)
            wp.pose.position.x = start_pose.pose.position.x + x_offset
            wp.pose.position.y = start_pose.pose.position.y + y_offset
            wp.pose.position.z = z_command
            waypoints.append(wp)

            theta += dtheta
            r += dr

        # Execute the spiral as a Cartesian path
        if not self.robot.move_cartesian_path(waypoints):
            self.robot.node.get_logger().warning("Spiral path execution failed")
            self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
            self.robot.set_speed_scaling(old_vel, old_acc)
            return False

        # Check if the EE dropped below the plate (indicating hole found)
        current_pose = self.robot.get_ee_pose()
        if current_pose is not None and current_pose.pose.position.z < z_plate - 0.005:
            self.robot.node.get_logger().info("Hole found! Z dropped below plate.")
            self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
            self.robot.set_speed_scaling(old_vel, old_acc)
            return True

        self.robot.node.get_logger().warning("Spiral search failed to find hole.")
        self.robot.set_cartesian_stiffness(STIFFNESS_DEFAULT)
        self.robot.set_speed_scaling(old_vel, old_acc)
        return False
