#!/bin/bash

# Ensure cleanup on script exit (including terminal close)
cleanup() {
    echo "Cleaning up processes..."
    kill -INT $ROS_PID 2>/dev/null
    kill $OLLAMA_PID $UI_PID 2>/dev/null
    pkill -P $UI_PID 2>/dev/null
    fuser -k 3000/tcp 2>/dev/null
}
trap cleanup EXIT INT TERM HUP

# Clean up any lingering processes from previous dirty exits
echo "Cleaning up any old processes..."
fuser -k 3000/tcp 2>/dev/null
killall -9 ros2_control_node move_group robot_state_publisher realsense2_camera_node realsense2_camera rviz2 ros2 2>/dev/null

# Check if ur5_mia_bridge_docker is running and shut it down
BRIDGE_DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/ur5_mia_bridge_docker" 2>/dev/null && pwd)"
if [ -d "$BRIDGE_DOCKER_DIR" ]; then
    if docker compose -f "$BRIDGE_DOCKER_DIR/docker-compose.yml" --project-directory "$BRIDGE_DOCKER_DIR" ps -q 2>/dev/null | grep -q .; then
        echo "ur5_mia_bridge_docker is running. Shutting it down..."
        docker compose -f "$BRIDGE_DOCKER_DIR/docker-compose.yml" --project-directory "$BRIDGE_DOCKER_DIR" down
    fi
fi

sleep 1

# Start Ollama
ollama serve &
OLLAMA_PID=$!

# Start UI
(
    cd ./orms_ui
    exec npm start
) &
UI_PID=$!

# Start ROS in the foreground
#cd ./orms_ws/orms_backend
#source ~/.bashrc
export ROS_DOMAIN_ID=0
source ./venv/bin/activate

#python -m colcon build --packages-select agent_server_interfaces agent_server orms_lib_interfaces

source ./install/setup.sh

# Start ROS in background and wait, so bash can reliably trap signals
ros2 launch agent_server manager.launch.xml &
ROS_PID=$!

wait $ROS_PID
