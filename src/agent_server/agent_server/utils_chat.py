#!/usr/bin/env python3
from __future__ import annotations
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

import random
from typing import Dict, Any, List, Literal
from enum import Enum

import asyncio
from pydantic import BaseModel, Field, field_validator
from pydantic_ai import Agent
from pydantic_ai.messages import ModelRequest, ModelResponse, UserPromptPart, TextPart
from pydantic_ai.models.openai import OpenAIModel
from pydantic_ai.providers.ollama import OllamaProvider
from pydantic_ai.exceptions import UnexpectedModelBehavior

from agent_server_interfaces.action import ChatAsk
import json
import yaml

from agent_server.utils_taskboard import TASKS, fill_tasks
from agent_server.prompts import PLANNER_PROMPT, REPORTER_PROMPT, CHATTER_PROMPT, BASE_URL, MODEL

# This code is here because the tasks will not be fillede else
package_dir = get_package_share_directory("agent_server")
config_path = os.path.join(package_dir, "config", "params.yaml")
with open(config_path) as f:
	data = yaml.safe_load(f)
fill_tasks(data)



ActionEnum = Enum(
	"ActionEnum",
	{"direct_answer": "direct_answer", **{task.id: task.id for task in TASKS}}
)
class RobotAction(BaseModel):
	action: ActionEnum
	parameters: dict[str, Any] = Field(default_factory=dict)
	
class ExecutionPlan(BaseModel):
	steps: List[RobotAction]

class ActionResult(BaseModel):
	action: str
	success: bool
	message: str

def get_model():
	return OpenAIModel(
		model_name=MODEL,
		provider=OllamaProvider(
			base_url=BASE_URL
		),
	)

def get_planner_agent():
	return Agent(
		model=get_model(),
		output_type=ExecutionPlan,
		system_prompt=PLANNER_PROMPT,
		retries=3
	)

def get_reporter_agent():
	return Agent(
		model=get_model(),
		system_prompt=REPORTER_PROMPT,
	)

def get_chatter_agent():
	return Agent(
		model=get_model(),
		system_prompt=CHATTER_PROMPT,
	)

def reporter_prompt(user_input:str, execution_results) -> str:
	return f"""
User request:
{user_input}

Execution results:
{execution_results}

Generate a concise response for the user.
"""


def execute_action(action: RobotAction) -> ActionResult:
	"""
	Simulated robot execution.
	Replace this with ROS2 calls later.
	"""

	if action.action == "error":
		return ActionResult(action="generate plan", success=False, message=action.parameters.get("message", "Unknown error"))
	try:
		print(f"\n[EXECUTOR] Running: {action.action}")
		print(f"[EXECUTOR] Parameters: {action.parameters}")
		success = random.random() > 0.1
		if success:
			return ActionResult(action=action.action, success=True, message=f"{action.action} executed successfully")
		return ActionResult(action=action.action, success=False, message=f"{action.action} failed due to simulated error")
	except Exception as e:
		return ActionResult(action="execute action", success=False, message=str(e))

def generate_plan(user_input:str, logger) -> ExecutionPlan:
	try:
		logger.info(f"Address: {BASE_URL}, Model: {MODEL}")
		plan_result = get_planner_agent().run_sync(user_input)
		plan_json = json.loads(plan_result.response.parts[0].args)
		plan = ExecutionPlan(**plan_json)
		if plan.steps is None or len(plan.steps) == 0 or (len(plan.steps) == 1 and plan.steps[0].action.value == "direct_answer"):
			logger.info(f"User asked a direct question, generating answer without execution. Plan: {plan.steps}")
			plan.steps = []
		logger.info(f"Generated plan: {plan}")
		return plan
	except Exception as e:
		if isinstance(e, UnexpectedModelBehavior):
			return ExecutionPlan(steps=[])
		else:
			raise e
		
async def direct_answer(history, user_input, goal_handle, logger) -> ChatAsk.Result:
	full_reply = ""
	logger.info(f"User asked a direct question, generating answer without execution.")
	async with get_chatter_agent().run_stream(user_input, message_history=history) as result:
		async for text in result.stream_text(delta=True):
			if goal_handle.is_cancel_requested:
				goal_handle.canceled()
				logger.info("Goal was canceled, stopping the response stream.")
				return ChatAsk.Result(success=False, full_answer="")
			full_reply += text
			feedback_msg = ChatAsk.Feedback()
			feedback_msg.partial_answer = text
			goal_handle.publish_feedback(feedback_msg)
	history.append(ModelRequest(parts=[UserPromptPart(content=user_input)]))
	history.append(ModelResponse(parts=[TextPart(content=full_reply)]))
	logger.info(f"Model answered : {full_reply}")
	goal_handle.succeed()
	return ChatAsk.Result(success=True, full_answer=full_reply)

async def report_execution(user_input:str, execution_results, goal_handle, history, logger) -> ChatAsk.Result:
	full_reply = ""
	reporter_input = reporter_prompt(user_input, execution_results)
	async with get_reporter_agent().run_stream(reporter_input) as result:
		async for text in result.stream_text(delta=True):
			if goal_handle.is_cancel_requested:
				goal_handle.canceled()
				logger.info("Goal was canceled, stopping the response stream.")
				return ChatAsk.Result(success=False, full_answer="")
			full_reply += text
			feedback_msg = ChatAsk.Feedback()
			feedback_msg.partial_answer = text
			goal_handle.publish_feedback(feedback_msg)
	history.append(ModelRequest(parts=[UserPromptPart(content=user_input)]))
	history.append(ModelResponse(parts=[TextPart(content=full_reply)]))
	logger.info(f"Model reported : {full_reply}")
	goal_handle.succeed()
	return ChatAsk.Result(success=True, full_answer=full_reply)
