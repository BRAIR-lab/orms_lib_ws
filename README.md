# ORMS: Open Robotics Manipulation Skills Library

**ORMS Library** (Open Robotics Manipulation Skills Library) is a comprehensive ROS 2-based software stack designed for the euROBIN Task Board Challenge. It provides a modular, multi-tier architecture to bridge high-level cognitive orchestration with low-level physical robotic control.

## System Architecture

The workspace is divided into three primary components:

1. **`orms_lib` (Robotic Manipulation Engine)**
   The core execution engine featuring a 4-tier hierarchical architecture:
   - **Interfaces**: Hardware abstraction layer utilizing a Factory pattern to support multiple robots (e.g., Franka FR3, UR5e) and grippers.
   - **Skills**: Reusable sensorimotor primitives (e.g., force-guarded moves, spiral searches).
   - **Tasks**: High-level execution logic for specific task board components (e.g., pressing buttons, opening doors).
   - **Nodes**: The `TaskServerNode` dynamically loads configurations and exposes tasks as individual ROS 2 Action Servers.

2. **`agent_server` (Orchestration & LLM Integration)**
   The high-level cognitive orchestrator. It acts as the bridge between the user interface and the physical robot:
   - Evaluates natural language user requests utilizing local LLMs via Ollama (e.g., `qwen3:4b-instruct`).
   - Uses `pydantic_ai` to safely enforce strict JSON-schema generation for deterministic task sequencing.
   - Manages the lifecycle of underlying hardware drivers (MoveIt, ROS 2 controllers) across different workspaces dynamically.

3. **`orms_ui` (Web User Interface)**
   A modern, responsive React-based web dashboard.
   - Leverages the Mantine component library and `react-grid-layout` for a fully customizable, drag-and-drop telemetry dashboard.
   - Communicates with the ROS 2 backend via `roslibjs` and `rosbridge_server`.
   - Bypasses ROS 2 for high-frequency task board telemetry (e.g., temperature, button counts), streaming directly over dedicated WebSockets.

## Getting Started

Please refer to the [Installation Guide](install.md) for step-by-step instructions on setting up the environment, compiling the ROS 2 workspaces, installing the UI dependencies, and downloading the local LLM.

## Usage

To launch the complete euROBIN software stack (backend, agent server, and UI), run the provided startup script from the workspace root:

```bash
./start_eurobin.sh
```

Navigate to `http://localhost:3000` (or the port indicated in the terminal) in your browser to access the operator dashboard.
