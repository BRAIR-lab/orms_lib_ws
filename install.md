# Installation Guide

Follow these steps to set up the `orms_lib_ws` workspace, including the ROS 2 backend, micro-ROS agent, UI frontend, and the local LLM.

## Prerequisites
- **ROS 2 Jazzy** (Ubuntu 24.04)
- **Python 3.12** with `venv`
- **Node.js & npm** (for the UI)
- **Ollama** (for local LLM inference)
- **vcstool** (`sudo apt install python3-vcstool`)
- **franka_ros2**: It is assumed that the Franka ROS 2 packages are already set up in a separate workspace (e.g., `~/franka_ros2_ws`) according to the [official documentation](https://github.com/frankarobotics/franka_ros2).

## 1. Clone the Workspace
Clone the main repository and navigate into the workspace directory:

```bash
git clone https://github.com/BRAIR-lab/orms_lib_ws.git
cd orms_lib_ws
```

## 2. Fetch Dependencies
Fetch the external repositories (perception pipelines) into the `src` directory using `vcstool`:

```bash
vcs import src < orms.repos
```

## 3. Install micro-ROS Agent
The system requires `micro-ROS` to communicate with the task board hardware.

```bash
# Update package lists and rosdep
sudo apt update
rosdep update

# Install micro-ROS messages
sudo apt install -y ros-$ROS_DISTRO-micro-ros-msgs

# Create and build the micro-ROS agent workspace
mkdir -p microros_agent_ws/src
cd microros_agent_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ROS-Agent.git src/micro-ROS-Agent
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
cd ..
```

*(Note: `rosbridge_suite` is also required. Install it via `sudo apt install -y ros-$ROS_DISTRO-rosbridge-suite` if you haven't already).*

## 4. Build the ROS 2 Workspace
We use a Python virtual environment to manage dependencies safely.

```bash
# Set up the virtual environment
./create_venv.sh
source venv/bin/activate

# Build the workspace
python3 -m colcon build
```

## 5. Setup the Web UI
The frontend is a standalone React application. Clone it into the workspace and install its dependencies:

```bash
git clone https://github.com/BRAIR-lab/orms_ui.git
cd orms_ui
sudo apt install npm -y
npm install
cd ..
```

## 6. Setup Local LLM (Ollama)
The `agent_server` relies on Ollama for local, offline LLM inference.

```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.com/install.sh | sh

# Download and run the default model
ollama pull qwen3:4b-instruct
```
> **Note:** After the model download completes, type `/bye` to exit the chat prompt.
> If you wish to use a different model (e.g., `granite4.1:8b`), download it via Ollama and update the model name in `src/agent_server/config/params.yaml`.

## 7. Running the System
Once everything is installed and built, you can start the entire stack from the main workspace folder:

```bash
./start_eurobin.sh
```
> **Note:** The first time you run this script, it may take a little longer to start as `npm start` prepares the development server.
