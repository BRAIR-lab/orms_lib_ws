import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from orms_lib_interfaces.action import TaskAction

from orms_lib.interfaces.perception_interface import PerceptionInterface
from orms_lib.interfaces.factory import create_robot, create_gripper
import importlib
import yaml
import os
from ament_index_python.packages import get_package_share_directory

import time

class TaskServerNode(Node):
    def __init__(self):
        super().__init__('task_server_node')
        
        # Reentrant callback group allows action callbacks to run
        # concurrently with internal future completions
        self._action_cb_group = ReentrantCallbackGroup()
        
        # Declare parameters
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('config_file', 'orms_config_fr3.yaml')
        sim_mode = self.get_parameter('sim_mode').get_parameter_value().bool_value
        config_file = self.get_parameter('config_file').get_parameter_value().string_value

        if sim_mode:
            self.get_logger().info("Starting in SIMULATION MODE")
        else:
            self.get_logger().info("Starting in REAL ROBOT MODE")

        # Load Config — config_file is relative to the package's share/config dir,
        # or an absolute path if the user passes one.
        if os.path.isabs(config_file):
            config_path = config_file
        else:
            pkg_share = get_package_share_directory('orms_lib')
            config_path = os.path.join(pkg_share, 'config', config_file)

        self.get_logger().info(f"Loading config: {config_path}")
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Initialize interfaces from config
        self.robot = create_robot(self, self.config['robot']['type'])
        self.gripper = create_gripper(self, self.config['gripper']['type'])
        self.perception = PerceptionInterface(self)
        
        # Instantiate task logic classes dynamically
        self.tasks = {}
        tasks_pkg = self.config.get('tasks_package', 'orms_lib.tasks')
        for task_name, task_cfg in self.config['tasks'].items():
            module_name = f"{tasks_pkg}.{task_cfg['file']}"
            module = importlib.import_module(module_name)
            
            # Find the TaskBase subclass in the module
            from orms_lib.tasks.task_base import TaskBase
            task_class = None
            for attr_name in dir(module):
                attr = getattr(module, attr_name)
                if isinstance(attr, type) and issubclass(attr, TaskBase) and attr is not TaskBase:
                    task_class = attr
                    break
            
            if task_class is None:
                self.get_logger().error(f"No TaskBase subclass found in {module_name}")
                continue
                
            params = task_cfg.get('params', {})
            self.tasks[task_name] = task_class(self, self.robot, self.gripper, self.perception, sim_mode, **params)
            self.get_logger().info(f"Loaded task: {task_name}")
        
        # Create individual action servers for each task
        self.action_servers = []
        for task_name in self.tasks.keys():
            server = ActionServer(
                self,
                TaskAction,
                f'/orms/{task_name}',
                execute_callback=self._create_execute_callback(task_name),
                callback_group=self._action_cb_group
            )
            self.action_servers.append(server)
            self.get_logger().info(f"Action server started for: {task_name}")

    def _create_execute_callback(self, task_name):
        def execute_callback(goal_handle):
            self.get_logger().info(f"Executing task: {task_name}")
            start_time = time.time()
            
            task_instance = self.tasks[task_name]
            success = task_instance.execute(goal_handle)
            
            result = TaskAction.Result()
            result.success = success
            result.execution_time_sec = time.time() - start_time
            if success:
                result.message = f"{task_name} completed successfully."
                goal_handle.succeed()
            else:
                result.message = f"{task_name} failed."
                goal_handle.abort()
                
            return result
            
        return execute_callback

def main(args=None):
    rclpy.init(args=args)
    node = TaskServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
