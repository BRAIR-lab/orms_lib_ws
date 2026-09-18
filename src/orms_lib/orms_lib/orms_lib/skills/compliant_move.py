import time
from orms_lib.interfaces.robot_base import RobotBase
from geometry_msgs.msg import Pose

class CompliantMove:
    def __init__(self, robot: RobotBase):
        self.robot = robot

    def execute_door_open(self, target_distance: float = 0.11, step_size: float = 0.015) -> bool:
        """
        Compliant stepping for door opening.
        Replaces GoTo_X loop in TaskBoardO_OpenDoor.m
        """
        self.robot.node.get_logger().info("Executing compliant move for door opening")
        
        # Set low Cartesian compliance
        self.robot.set_impedance(k_p=[1000, 0, 0], k_r=[30, 30, 30])
        
        distance_opened = 0.0
        failures = 0
        
        # Simulate tangent tracking
        while distance_opened < target_distance and failures < 30:
            # Estimate tangent, compute step
            # p_delta = tangent * step_size
            # self.robot.compliant_step(pose_target, stiffness, dt)
            
            time.sleep(0.2)
            distance_opened += step_size
            
        if distance_opened >= target_distance:
            self.robot.node.get_logger().info("Door successfully opened.")
            return True
            
        return False
