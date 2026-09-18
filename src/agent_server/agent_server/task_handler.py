#!/usr/bin/env python3
from typing import List
import asyncio
import threading

import yaml

import rclpy
from rclpy.node import Node

from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from robothon_taskboard_msgs.action import ExecuteTask
from orms_lib_interfaces.action import TaskAction
from agent_server_interfaces.action import ChatAsk
from agent_server_interfaces.srv import SetList, StartTask, FinishTime
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from action_msgs.msg import GoalStatus
from robothon_taskboard_msgs.msg import Task

from agent_server.utils_chat import generate_plan, direct_answer, report_execution
from agent_server.utils_taskboard import make_task_for_task_board, check_task_order, task_id_from_name, task_from_name, fill_tasks



class TaskHandler(Node):

	def __init__(self):
		super().__init__('minimal_publisher')

		self.declare_parameter("config_file", "")
		config_path = self.get_parameter("config_file").get_parameter_value().string_value
		self.get_logger().info(f"Loading config from: {config_path}")
		with open(config_path) as f:
			data = yaml.safe_load(f)
		fill_tasks(data)
		
		self.param_client = self.create_client(SetParameters, '/config_node/set_parameters')
		while not self.param_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Waiting for node_b parameter service...')

		self.srv2 = self.create_service(SetList, 'set_list', self.set_list)
		self.srv3 = self.create_service(StartTask, 'send_list', self.send_list)
		self.srv4 = self.create_service(StartTask, 'abort_task', self.abort_task)
		self.srv5 = self.create_service(FinishTime, 'finish_time', self.get_final_result)
		self.finish_time = None
		self._goal_handle = None
		self.current_task = Task()
		self.sequence_id = []
		self.cancel_execution = lambda *args: None
		self.rerun_pose_estimation_client = self.create_client(Trigger, '/tum_tb_perception2/rerun_pose_estimation')


		self._action_server = ActionServer( self, ChatAsk, '/web_chat/chat_ask', self.chat_ask,
									 callback_group=ReentrantCallbackGroup())

		self.task_sender = ActionClient(self, ExecuteTask, 'taskboard_execute_task')# to send tasks to the taskboard
		self.get_logger().info(f"Waiting for MicroROS (actually skipping beacuse there is a probvlem with uros)")
		# self.task_sender.wait_for_server()
		self.get_logger().info("MicroROS ready")

		# self.localize_client = self.create_client(LocalizeTaskboard, '/localize_taskboard')
		# while not self.localize_client.wait_for_service(timeout_sec=1.0):
		# 	self.get_logger().debug('Waiting for localize_taskboard service...')

	def _get_action_client(self, task_name: str) -> ActionClient:
		"""One action server per action — cache a client per task name."""
		if not hasattr(self, '_action_clients'):
			self._action_clients = {}
		if task_name not in self._action_clients:
			self._action_clients[task_name] = ActionClient(self, TaskAction, f'/orms/{task_name}')
		return self._action_clients[task_name]

	def _detect_taskboard(self) -> tuple[bool, str]:
		if not self.rerun_pose_estimation_client.wait_for_service(timeout_sec=5.0):
			self.get_logger().error("Pose estimation service unavailable after waiting 5 seconds")
			return (False, "Pose estimation service unavailable")

		service_future = self.rerun_pose_estimation_client.call_async(Trigger.Request())
		service_event = threading.Event()
		service_future.add_done_callback(lambda _: service_event.set())
		if not service_event.wait(timeout=15.0):
			self.get_logger().error("Pose estimation service did not respond within 15 seconds")
			return (False, "Pose estimation service timed out")

		try:
			service_response = service_future.result()
		except Exception as exc:
			self.get_logger().error(f"Pose estimation service call failed: {exc}")
			return (False, "Pose estimation service call failed")

		if not service_response.success:
			message = service_response.message or "Pose estimation failed"
			self.get_logger().error(f"Pose estimation failed: {message}")
			return (False, message)

		self.get_logger().info("Pose estimation rerun completed")
		return (True, "Pose estimation rerun completed")

	def send_task_sequence(self, task_names: list) -> list[tuple[bool, str]]:
		"""Given a list of task name strings, call the action server for each one
		(one server per action), waiting for each to finish before sending the next."""
		results = []
		for task_name in task_names:
			if(type(task_name) == str):
				id_name = task_name
			else:
				id_name = task_name.action.name
			if id_name == "detect_taskboard":
				success, message = self._detect_taskboard()
				results.append((success, message))
				if not success:
					break
				continue
			client = self._get_action_client(id_name)
			if not client.wait_for_server(timeout_sec=5.0):
				self.get_logger().error(f"Action server for '{task_name}' not available")
				results.append((False, "ROS 2 Action server unavailable"))
				break
				
			goal_msg = TaskAction.Goal()
			goal_msg.task_name = id_name
			goal_msg.param_keys = []
			goal_msg.param_values = []
			
			def feedback_cb(feedback_msg, t_name=id_name):
				fb = feedback_msg.feedback
				self.get_logger().info(f"[{t_name}] phase={fb.current_phase} progress={fb.progress:.2f}")
				
			future = client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
			self.cancel_execution = lambda *args: client._cancel_goal_async(goal_msg)
			
			# Wait for goal to be accepted using a Threading Event (safe for MultiThreadedExecutors)
			event = threading.Event()
			future.add_done_callback(lambda f: event.set())
			event.wait()
			
			goal_handle = future.result()
			if not goal_handle.accepted:
				self.get_logger().warn(f"Goal rejected for task {task_name}")
				results.append((False, "Goal rejected"))
				self.cancel_execution = lambda *args: None
				break
				
			# Wait for the final result securely
			result_future = goal_handle.get_result_async()
			result_event = threading.Event()
			result_future.add_done_callback(lambda f: result_event.set())
			result_event.wait()
			
			result_response = result_future.result()
			result = result_response.result
			
			self.get_logger().info(f"[{task_name}] success={result.success} msg={result.message}")
			results.append((result.success, result.message))
			if not result.success:
				self.cancel_execution = lambda *args: None
				break
		self.cancel_execution = lambda *args: None	
		return results

	def chat_ask(self, goal_handle):
		self.get_logger().info("Received goal")
		# reset the chat
		if goal_handle.request.question == "":
			self.history = []
			goal_handle.succeed()
			return ChatAsk.Result(success=True, full_answer="")
			
		try:
			# ask if generate a sequence of tasks or to ansewr directly
			plan = generate_plan(goal_handle.request.question, self.get_logger())
		except Exception as e:
			self.get_logger().error(f"Error generating plan: {e}")
			goal_handle.abort()
			return ChatAsk.Result(success=False, full_answer=f"Error handling request: {e}")
			
		if not plan.steps or len(plan.steps) == 0:  
			# Direct factual question - Wrap in asyncio.run to give pydantic_ai a proper event loop
			answer = asyncio.run(direct_answer(self.history, goal_handle.request.question, goal_handle, self.get_logger()))
			return answer
		else: 
			# Execute generated plan natively
			execution_results = self.send_task_sequence(plan.steps)
			
			# report_execution utilizes LLM, run in its own event loop
			answer = asyncio.run(report_execution(goal_handle.request.question, execution_results, goal_handle, self.history, self.get_logger()))
			return answer

	def set_list(self, request, response):
		self.get_logger().info(str(request.data))
		response.success, response.message = check_task_order(request.data)
		self.change_node_b_parameter('board.selected_list', request.data)
		task = Task()
		task.name = "UI tasks"
		task_with_nums = [task_from_name(task_name, float_value=value) for task_name, value in zip(request.data, request.values)]
		steps = make_task_for_task_board(task_with_nums)
		task.steps.extend(steps)
		self.sequence_id = [task_id_from_name(task_name) for task_name in request.data]
		self.current_task = task
		return response

	# Send list to the taskboard and start a trial
	# also in case the 
	def send_list(self, request, response):
		# send the task to taskboards
		goal_msg = ExecuteTask.Goal()
		goal_msg.human_task = False
		goal_msg.task = self.current_task
		self.get_logger().info('Sending goal request...')
		self._send_goal_future = self.task_sender.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
		self._send_goal_future.add_done_callback(self.goal_response_callback)
		if(not request.interactive):
			# start tasks in teh correct order in the background
			threading.Thread(
				target=self.send_task_sequence,
				args=(self.sequence_id,),
				daemon=True
			).start()
		return response
	def abort_task(self, request, response):
		if self._goal_handle is None:
			self.get_logger().info("No active goal to cancel")
			# Always return a response object (rclpy requires a non-None response)
			return response
		self.get_logger().info("Cancelling current task...")
		self._cancel_future = self._goal_handle.cancel_goal_async()
		self._cancel_future.add_done_callback(self.cancel_callback)
		self.cancel_execution()
		return response
	def feedback_callback(self, feedback):
		message = 'Feedback: {0}: '.format(feedback.feedback.elapsed_time)
		self.get_logger().debug('Received feedback: {0}'.format(message))
	def get_result_callback(self, future):
		result = future.result().result
		status = future.result().status
		if status == GoalStatus.STATUS_SUCCEEDED:
			self.get_logger().info('Goal succeeded! Result: {0}'.format(result.finish_time))
			self.finish_time = result.finish_time
		else:
			self.get_logger().info('Goal failed with status: {0}'.format(status))
	def goal_response_callback(self, future):
		goal_handle = future.result()
		self._goal_handle = goal_handle
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected')
			return
		self.get_logger().info('Goal accepted')
		self._get_result_future = goal_handle.get_result_async()
		self._get_result_future.add_done_callback(self.get_result_callback)
	def cancel_callback(self, future):
		cancel_response = future.result()
		if len(cancel_response.goals_canceling) > 0:
			self.finish_time = None
			self.get_logger().info("Task cancelled")
		else:
			self.get_logger().warning("Task could not be cancelled")
		self._goal_handle = None

	def get_final_result(self, request, response):
		if self.finish_time is not None:
			# `FinishTime_Response.time` is an int32. Return milliseconds (ms).
			try:
				sec = int(self.finish_time.sec)
				nsec = int(self.finish_time.nanosec)
				ms = sec * 1000 + (nsec // 1000000)
			except Exception:
				# fallback: if finish_time is already an integer (assumed ms)
				ms = int(self.finish_time)
			response.time = ms
			response.success = True
			self.get_logger().info(f"Returning finish time: {response.time} ms")
		else:
			response.success = False
			self.get_logger().info("No finish time available")
		return response

	
	def change_node_b_parameter(self, param_name, param_value_list):
		# Create the parameter
		param = Parameter()
		param.name = param_name
		param.value = ParameterValue()
		
		# Set the type and value for string array
		param.value.type = ParameterType.PARAMETER_STRING_ARRAY
		param.value.string_array_value = param_value_list  # e.g., ['item1', 'item2', 'item3']
		
		# Create and send the request
		request = SetParameters.Request()
		request.parameters = [param]
		
		future = self.param_client.call_async(request)
		future.add_done_callback(self.parameter_callback)
	
	def parameter_callback(self, future):
		result = future.result()
		if result.results[0].successful:
			self.get_logger().info('Parameter changed successfully')
		else:
			self.get_logger().error('Failed to change parameter')




def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = TaskHandler()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor(4)

    rclpy.spin(minimal_publisher, executor=executor)

    minimal_publisher.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
	main()