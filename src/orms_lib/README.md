# ORMS Library (`orms_lib`)

**orms_lib** (Open Robotics Manipulation Skills Library) is a ROS 2 package serving as the core Robotic Manipulation Engine for the euROBIN Task Board Challenge. It bridges high-level cognitive orchestration with low-level physical control, providing a modular architecture designed to support diverse robotic manipulators and custom task solutions.

## Architecture Overview

The system is split into two main ROS 2 packages:
- `orms_lib_interfaces`: Defines custom ROS 2 Actions (e.g., `TaskAction.action`).
- `orms_lib`: The core Python package containing the orchestration and manipulation logic.

### 4-Tier Hierarchical Design

The core logic of `orms_lib` is divided into four distinct tiers:
1. **Nodes (`nodes/`)**: E.g., `task_server_node.py`. The lifecycle manager that dynamically loads hardware configurations and instantiates individual ROS 2 Action Servers for every defined task.
2. **Tasks (`tasks/`)**: High-level execution logic for solving specific components of the task board (e.g., `PressButtonTask`, `OpenDoorTask`). Tasks inherit from `TaskBase` and string together skills and hardware commands.
3. **Skills (`skills/`)**: Reusable, closed-loop sensorimotor primitives (e.g., `ForceGuardedMove`, `SpiralSearch`) that abstract complex sensory interactions.
4. **Interfaces (`interfaces/`)**: A hardware abstraction layer utilizing a Factory pattern to load specific platforms (e.g., Franka FR3, UR5e). It provides a unified API (`move_cartesian`, `set_stiffness`, etc.) ensuring tasks remain hardware-agnostic.

## Configuration

Task and hardware assignments are dynamically driven by YAML configuration files (e.g., `orms_config_fr3.yaml` and `orms_config_ur5e.yaml` in the `config/` directory). 

The configuration file defines:
- **Robot and Gripper Types**: E.g., `franka_fr3`, `ur5e`. The Factory pattern dynamically instantiates the correct interface.
- **Perception System**: Target ML models (e.g., YOLO) and tracking logic.
- **Active Tasks**: A registry of tasks that the `TaskServerNode` will wrap as ROS 2 Action Servers.

## Adding a Novel Task

To add a custom manipulation task:
1. **Create the Task**: Write a Python class inheriting from `TaskBase` in `orms_lib/tasks/` (or your custom package). Implement the `execute(self, goal_handle)` method.
2. **Register in Config**: Add your task to `orms_config_fr3.yaml` under the `tasks:` section.
3. **Link Module**: Set the `file:` key to point to your Python module (e.g., `file: jsi.my_custom_task`).
4. **Execute**: The `TaskServerNode` will automatically expose it as a ROS Action Server (e.g., `/orms/<your_task_name>`).

## Usage

### 1. Build the packages
From the root of your workspace:
```bash
colcon build --packages-select orms_lib_interfaces orms_lib
```

### 2. Source the workspace
```bash
source install/setup.bash
```

### 3. Launch the System
To launch the entire ORMS stack (including the RealSense camera, perception pipeline, static TF transforms, and the ORMS Task Server), use the comprehensive `system_bringup.launch.py` file:

```bash
ros2 launch orms_lib system_bringup.launch.py
```
*(By default, this loads `orms_config_fr3.yaml` and sets up `yolo_pnp` perception. You can override this if a different hardware configuration is needed by passing the `orms_config_file` or `perception_type` parameters).*

Alternatively, if you only want to launch the pure `task_server` node without the cameras and perception pipelines (e.g., for testing):
```bash
ros2 launch orms_lib task_server.launch.py
```
