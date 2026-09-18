from typing import Any, Dict
import yaml
from robothon_taskboard_msgs.msg import TaskStep
from robothon_taskboard_msgs.msg import SensorMeasurement
from pathlib import Path

class Task():
	def __init__(self, name: str, id: str, description: str, sensor_name: str, type: str, success_value: Any, selectable: bool):
		self.name = name
		self.id = id
		self.description = description
		self.sensor_name = sensor_name
		self.type = type
		self.selectable = selectable
		self.success_value = success_value
	name: str
	id: str
	description: str
	sensor_name: str
	type: str
	success_value: Any
	selectable: bool

TASKS = []

# <param from="$(find-pkg-share agent_server)/config/params.yaml" />
def fill_tasks(data, include_special: bool = True):
	try:
		global TASKS
		TASKS.clear()
		params = data.get("/**", {}).get("ros__parameters", {})
		board = params.get("board", {})
		tasks = board.get("tasks", [])
		for task in tasks:
			task_name = task.get("name", "")
			task_id = task.get("id", "")
			task_description = task.get("description", "")
			task_sensor_name = task.get("sensor_name", "")
			if not include_special and task_sensor_name == "":
				continue
			task_type = task.get("type", "")
			task_success_value = task.get("success_value", None)
			task_selectable = task.get("selectable", False)
			TASKS.append(Task(name=task_name, id=task_id, description=task_description, sensor_name=task_sensor_name, type=task_type, success_value=task_success_value, selectable=task_selectable))
		return TASKS
	except Exception:
		raise RuntimeError(f"Failed to load tasks from yaml")


def task_from_name(task_name: str, float_value: float) -> Task:
	for task in TASKS:
		if task.name == task_name:
			if task.selectable:
				task.success_value = float_value
			return task
	raise ValueError(f"Task with name '{task_name}' not found. Valid task names are: {[task.name for task in TASKS]}")

def task_id_from_name(task_name: str) -> str:
	for task in TASKS:
		if task.name == task_name:
			return task.id
	raise ValueError(f"Task with name '{task_name}' not found. Valid task names are: {[task.name for task in TASKS]}")

def make_task_for_task_board(tasks:list[Task]) -> list[TaskStep]:
	steps = []
	for task in tasks:
		task_step = TaskStep()
		task_step.sensor_name = task.sensor_name
		task_step.type = TaskStep.TASK_STEP_TYPE_EQUAL
		if task.type == "bool":
			task_step.target.type = SensorMeasurement.SENSOR_MEASUREMENT_TYPE_BOOL
			task_step.target.bool_value.append(task.success_value)
		elif task.type == "float":
			task_step.target.type = SensorMeasurement.SENSOR_MEASUREMENT_TYPE_ANALOG
			task_step.target.analog_value.append(task.success_value)
			task_step.tolerance = 0.1
		steps.append(task_step)
	return steps

def check_task_order(tasks:list[str]) -> tuple[bool, str]:
	return True, 'Tasks are ready to run'