import subprocess
import signal
import os
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from robothon_taskboard_msgs.action import ExecuteTask

from agent_server_interfaces.srv import SetList
from agent_server_interfaces.srv import SetString
from agent_server_interfaces.msg import RosStatus

from ament_index_python.packages import get_package_prefix


class ManagerNode(Node):

	def __init__(self):
		super().__init__("manager_node")

		# Track multiple processes for a single robot
		self.current_processes = []
		self.current_robot = None

		self.service = self.create_service(SetString, "select_robot", self.select_robot_callback)

		self.publisher_ = self.create_publisher(RosStatus, 'ros_status', 10)
		self.timer = self.create_timer(1.0, self.timer_callback)

		self.task_sender = ActionClient(self, ExecuteTask, 'taskboard_execute_task') 
		self.cli2 = self.create_client(SetList, 'set_list') 

		self.basedir = self.get_basedir()
		if self.basedir:
			self.get_logger().info(f"Manager node started. Base directory resolved to: {self.basedir}")
		else:
			self.get_logger().error("Manager node failed to resolve base directory!")

	def get_basedir(self):
		try:
			prefix_path = get_package_prefix('agent_server')
			basedir_path = os.path.abspath(os.path.join(prefix_path, '..', '..', '..'))
			return basedir_path
		except Exception as e:
			self.get_logger().error(f"Could not find agent_server package path: {e}")
			return None

	def get_robot_config(self, robot, config_filename=None):
		"""Load robot configuration YAML (e.g. orms_config_fr3.yaml) from orms_lib config directory."""
		if not config_filename:
			if robot in ["franka", "fr3"]:
				config_filename = "orms_config_fr3.yaml"
			elif robot in ["ur5", "ur5e", "robot_b"]:
				config_filename = "orms_config_ur5e.yaml"
			else:
				config_filename = f"orms_config_{robot}.yaml"

		search_paths = []
		if os.path.isabs(config_filename):
			search_paths.append(config_filename)
		else:
			if self.basedir:
				search_paths.extend([
					os.path.join(self.basedir, "orms_lib_ws", "install", "orms_lib", "share", "orms_lib", "config", config_filename),
					os.path.join(self.basedir, "orms_lib_ws", "src", "orms_lib", "orms_lib", "config", config_filename),
				])

		for path in search_paths:
			if os.path.exists(path):
				try:
					with open(path, 'r') as f:
						config = yaml.safe_load(f) or {}
						self.get_logger().info(f"Loaded config for {robot} from: {path}")
						return config, config_filename
				except Exception as e:
					self.get_logger().error(f"Error reading config file {path}: {e}")
			else:
				self.get_logger().info(f"Path not exists: {path}")

		self.get_logger().warn(f"Config file '{config_filename}' not found for {robot}. Using fallback defaults.")
		return {}, config_filename

	def select_robot_callback(self, request, response):
		robot_req = request.data
		self.get_logger().info(f"Received request to select: {robot_req}")

		if not self.basedir:
			response.success = False
			response.message = "System error: Base directory not found."
			return response

		config_file = None
		if ":" in robot_req:
			robot, config_file = robot_req.split(":", 1)
		else:
			robot = robot_req

		if robot in ["franka", "fr3"]:
			response.success = self.start_robot("franka", config_file)
		elif robot == "robot_b":
			self.get_logger().info("Robot B selected")
			response.success = self.start_robot("robot_b", config_file)
		else:
			self.get_logger().error(f"Unknown robot: {robot}")
			response.success = False
			response.message = f"Unknown robot: {robot}"
			return response

		if response.success:
			response.message = f"Started {robot}"
		else:
			response.message = f"Failed to start {robot}"
		return response

	def start_robot(self, robot, config_filename=None):
		if self.current_robot == robot and len(self.current_processes) > 0:
			self.get_logger().info(f"{robot} is already running")
			return True

		self.stop_current_robot()

		# Load configuration for the robot from config file
		config, cfg_filename = self.get_robot_config(robot, config_filename)
		robot_cfg = config.get("robot", {})
		robot_ip = robot_cfg.get("robot_ip", "192.168.1.198")
		load_gripper = robot_cfg.get("load_gripper", True)
		load_gripper_str = "true" if load_gripper else "false"

		perception_cfg = config.get("perception", {})
		perception_type = perception_cfg.get("type", "yolo_pnp")
		use_pointcloud_z = perception_cfg.get("use_pointcloud_z", True)
		use_pointcloud_z_str = "true" if use_pointcloud_z else "false"

		camera_cfg = config.get("camera", {})
		if isinstance(camera_cfg, dict):
			camera_type = str(camera_cfg.get("type", camera_cfg.get("model", "d435"))).lower().strip()
		elif isinstance(camera_cfg, str):
			camera_type = camera_cfg.lower().strip()
		else:
			camera_type = "d435"

		self.get_logger().info(
			f"Starting {robot} with params from config ({cfg_filename}): "
			f"robot_ip={robot_ip}, load_gripper={load_gripper_str}, "
			f"perception_type={perception_type}, use_pointcloud_z={use_pointcloud_z_str}, camera_type={camera_type}"
		)

		# Define launch sequences as lists of dictionaries
		if robot in ["franka", "fr3"]:
			launch_steps = [
				{
					"workspace": "franka_ros2_ws",
					"package": "franka_fr3_moveit_config",
					"file": "moveit.launch.py",
					"args": f"robot_ip:={robot_ip} load_gripper:={load_gripper_str} use_rviz:=false"
				},
				{
					"workspace": "orms_lib_ws",
					"package": "orms_lib",
					"file": "system_bringup.launch.py",
					"args": f"orms_config_file:={cfg_filename} perception_type:={perception_type} use_pointcloud_z:={use_pointcloud_z_str} camera_type:={camera_type}"
				}
			]
		elif robot == "robot_b":
			launch_steps = [
				{
					"workspace": "UR5_dir",
					"package": "ur5_bringup",
					"file": "robot_b.launch.xml",
					"args": ""
				}
			]
		else:
			self.get_logger().error(f"No configuration defined for {robot}")
			return False

		# Execute each step sequentially
		for step in launch_steps:
			setup_file = os.path.join(self.basedir, step["workspace"], "install", "setup.bash")
			
			if not os.path.exists(setup_file):
				self.get_logger().error(f"Setup file not found: {setup_file}")
				self.stop_current_robot() # Cleanup any processes already started
				return False

			try:
				self.get_logger().info(f"Launching {step['file']} from {step['workspace']}")
				
				# Combine args cleanly and construct bash command
				bash_command = f"source {setup_file} && exec ros2 launch {step['package']} {step['file']} {step['args']}".strip()
				self.get_logger().info(f"Executing command: {bash_command}")
				process = subprocess.Popen(
					bash_command,
					shell=True,
					executable='/bin/bash',
					start_new_session=True
				)
				self.current_processes.append(process)
				time.sleep(2)  # Allow some time for the process to start

			except Exception as e:
				self.get_logger().error(f"Failed to start {robot} at step {step['file']}: {e}")
				self.stop_current_robot()
				return False

		self.current_robot = robot
		return True

	def stop_current_robot(self):
		if not self.current_processes:
			return
			
		self.get_logger().info(f"Stopping {self.current_robot} ({len(self.current_processes)} processes)")

		# Iterate in reverse to shut down higher-level nodes before base drivers
		for proc in reversed(self.current_processes):
			try:
				os.killpg(os.getpgid(proc.pid), signal.SIGINT)
				proc.wait(timeout=5)
				self.get_logger().info(f"Successfully stopped process {proc.pid}")
			except subprocess.TimeoutExpired:
				self.get_logger().warning(f"Process {proc.pid} did not stop gracefully, killing it forcefully")
				os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
			except Exception as e:
				self.get_logger().error(f"Error stopping process {proc.pid}: {e}")

		self.current_processes = []
		self.current_robot = None

	def timer_callback(self):
		# Native Python dictionary representing your data
		process_dict = {
			"mu_ros": self.task_sender.server_is_ready(),
			"robot": self.cli2.service_is_ready(),
			"orms_lib": len(self.current_processes) > 0
		}
		
		msg = RosStatus()
		# Separate the keys and values into parallel lists
		msg.names = list(process_dict.keys())
		msg.completed = list(process_dict.values())
		self.publisher_.publish(msg)

	def destroy_node(self):
		self.get_logger().info("Shutting down manager")
		self.stop_current_robot()
		super().destroy_node()

def main(args=None):
	rclpy.init(args=args)
	node = ManagerNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		node.get_logger().info("Keyboard interrupt received. Shutting down...")
	finally:
		node.get_logger().info("Cleaning up spawned robot processes...")
		node.stop_current_robot()
		node.destroy_node()
		if rclpy.ok():
			rclpy.shutdown()

if __name__ == "__main__":
	main()