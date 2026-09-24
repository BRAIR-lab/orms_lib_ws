# Installation Guide

Follow these steps to set up the `orms_lib_ws` workspace, including the ROS 2 backend, micro-ROS agent, UI frontend, and the local LLM.

## Prerequisites
- **ROS 2 Jazzy** (Ubuntu 24.04)
- **Python 3.12** with `venv`
- **Node.js & npm** (for the UI)
- **Ollama** (for local LLM inference)
- **vcstool** (`sudo apt install ros-dev-tools`)
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
sudo apt update
rosdep update
vcs import src < orms.repos
```

## 4. Install ROS 2 Dependencies
Install all ROS 2 system dependencies declared by packages in the workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 5. Build the ROS 2 Workspace
We use a Python virtual environment to manage dependencies safely.

```bash
# Set up the virtual environment
./create_venv.sh
source venv/bin/activate

# Build the workspace
python3 -m colcon build
```

## 6. Setup the Web UI
The frontend is a standalone React application. Clone it into the workspace and install its dependencies:

```bash
git clone https://github.com/BRAIR-lab/orms_ui.git
cd orms_ui
sudo apt install npm -y
npm install
cd ..
```

## 7. Setup Local LLM (Ollama)
The `agent_server` relies on Ollama for local, offline LLM inference.

```bash
# Install Ollama (if not already installed)
curl -fsSL https://ollama.com/install.sh | sh

# Download and run the default model
ollama pull qwen3:4b-instruct
```
> If you wish to use a different model (e.g., `granite4.1:8b`), download it via Ollama and update the model name in `src/agent_server/config/params.yaml`.

## 8. Running the System
Once everything is installed and built, you can start the entire stack from the main workspace folder:

```bash
./start_eurobin.sh
```
> **Note:** The first time you run this script, it may take a little longer to start as `npm start` prepares the development server.
