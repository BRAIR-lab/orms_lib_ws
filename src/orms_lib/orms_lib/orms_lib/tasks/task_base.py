from abc import ABC, abstractmethod
from orms_lib.interfaces.robot_base import RobotBase
from orms_lib.interfaces.gripper_base import GripperBase
from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib_interfaces.action import TaskAction
import rclpy
from rclpy.action.server import ServerGoalHandle

class TaskBase(ABC):
    def __init__(self, node, robot: RobotBase, gripper: GripperBase, perception: PerceptionInterface, sim_mode: bool = False, **kwargs):
        self.node = node
        self.robot = robot
        self.gripper = gripper
        self.perception = perception
        self.sim_mode = sim_mode
        
    @abstractmethod
    def execute(self, goal_handle: ServerGoalHandle) -> bool:
        """
        Execute the task. 
        Should return True if successful, False otherwise.
        """
        pass
        
    def publish_feedback(self, goal_handle: ServerGoalHandle, phase: str, progress: float):
        feedback_msg = TaskAction.Feedback()
        feedback_msg.current_phase = phase
        feedback_msg.progress = float(progress)
        goal_handle.publish_feedback(feedback_msg)
