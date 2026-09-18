from abc import ABC, abstractmethod
import rclpy

class GripperBase(ABC):
    """
    Abstract base class for gripper hardware interfaces.
    """
    def __init__(self, node: rclpy.node.Node):
        self.node = node

    @abstractmethod
    def move(self, width: float, speed: float = 0.1) -> bool:
        pass

    @abstractmethod
    def close(self, width: float = 0.0, speed: float = 0.1, force: float = 10.0, 
              epsilon_inner: float = 0.005, epsilon_outer: float = 0.005) -> bool:
        pass

    @abstractmethod
    def open(self) -> bool:
        pass

    @abstractmethod
    def get_width(self) -> float:
        pass
