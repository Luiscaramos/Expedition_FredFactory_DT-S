#!/bin/bash

# 1. Provide GUI access to the Docker container (for RViz/Gazebo)
# This allows the container to open windows on your host screen
xhost +local:docker > /dev/null 2>&1

# 2. Define the container name and local directory
CONTAINER_NAME="ros2_humble_container"
# This finds the directory where this script is saved
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"

# 3. Detect Hardware: Check for NVIDIA GPU to set rendering mode
if nvidia-smi > /dev/null 2>&1; then
    # Your computer (Ubuntu 22.04 with NVIDIA)
    export RENDER_MODE="0" 
    echo "-------------------------------------------------------"
    echo "NVIDIA GPU detected. Using Hardware Acceleration."
else
    # Lab computer (Ubuntu 24.04 without GPU)
    export RENDER_MODE="1" 
    echo "-------------------------------------------------------"
    echo "No NVIDIA GPU detected. Forcing Software Rendering (LLVMpipe)."
fi

# 4. Check if the container is already running
if [ ! "$(sudo docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Container '$CONTAINER_NAME' is not running."
    echo "Starting it now using Docker Compose..."
    echo "-------------------------------------------------------"
    
    cd "$REPO_DIR"
    
    # Use -E to pass the RENDER_MODE variable to sudo environment
    LIBGL_ALWAYS_SOFTWARE=$RENDER_MODE sudo -E docker compose up -d
    
    # Brief pause to let the container initialize
    sleep 2
fi

# 5. Open a new bash session inside the container
# This version automatically sources the xArm drivers
echo "Successfully connected to $CONTAINER_NAME."
echo "Location: /ros2_ws"
echo "-------------------------------------------------------"

sudo docker exec -it -e LIBGL_ALWAYS_SOFTWARE=$RENDER_MODE $CONTAINER_NAME bash -c "
    source /opt/ros/humble/setup.bash && \
    source /opt/xarm_ws/install/setup.bash && \
    cd /ros2_ws && \
    exec bash"