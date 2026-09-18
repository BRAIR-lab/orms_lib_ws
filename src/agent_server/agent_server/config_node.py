# frontend_config_node.py
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from agent_server.utils_taskboard import fill_tasks
import yaml
import os
from pathlib import Path

class FrontendConfig(Node):
    def __init__(self):
        super().__init__('frontend_config')
        self.get_logger().info("Parameter initialized")

        self.declare_parameter("config_file", "")
        config_path = self.get_parameter("config_file").get_parameter_value().string_value
        with open(config_path) as f:
            data = yaml.safe_load(f)
        params = data["/**"]["ros__parameters"]

        self.declare_parameter('stream.type', params["stream"]["type"])
        self.declare_parameter('stream.names', params["stream"]["names"])

        self.declare_parameter('task.type', params["task"]["type"])
        self.declare_parameter('task.name', params["task"]["name"])

        self.declare_parameter('board.name', params["board"]["name"])
        self.declare_parameter('board.type', params["board"]["type"])

        self.declare_parameter('board.selected_list', [''])
        TASKS = fill_tasks(data, include_special=False)
        list_of_task_names = [task.name for task in TASKS]
        self.get_logger().info(f"List of task names: {list_of_task_names}")
        self.declare_parameter('board.list', list_of_task_names)
        selctable_tasks = [task.name for task in TASKS if task.selectable]
        self.declare_parameter('board.selectable_target_list', selctable_tasks)
        self.get_logger().info(f"List of selectable task names: {selctable_tasks}")

def main():
    rclpy.init()
    node = FrontendConfig()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
