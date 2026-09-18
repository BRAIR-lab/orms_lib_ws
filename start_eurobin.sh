#!/bin/bash

# Start Ollama
ollama serve &
OLLAMA_PID=$!

# Start UI
(
    cd ./orms_ui
    exec npm start
) &
UI_PID=$!

# Ensure cleanup on script exit (including terminal close)
cleanup() {
    echo "Cleaning up processes..."
    kill -INT $ROS_PID 2>/dev/null
    kill $OLLAMA_PID $UI_PID 2>/dev/null
    pkill -P $UI_PID 2>/dev/null
}
trap cleanup EXIT INT TERM HUP

# Clean up any lingering ROS 2 zombie processes from previous dirty exits
echo "Cleaning up any old ROS 2 processes..."
killall -9 ros2_control_node move_group robot_state_publisher realsense2_camera_node realsense2_camera rviz2 ros2 2>/dev/null
sleep 1

# Start ROS in the foreground
#cd ./orms_ws/orms_backend
#source ~/.bashrc
source ./venv/bin/activate

#python -m colcon build --packages-select agent_server_interfaces agent_server orms_lib_interfaces

source ./install/setup.sh

# Start ROS in background and wait, so bash can reliably trap signals
ros2 launch agent_server manager.launch.xml &
ROS_PID=$!

wait $ROS_PID
