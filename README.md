# Expedition_FredFactory_DT-S
Expedition digital twin and Shadow 

# ROS 2 Humble xArm Development Environment

This repository contains a Dockerized development environment for ROS 2 Humble, specifically configured for xArm robotics projects. It is designed to work seamlessly across different host operating systems (Ubuntu 22.04 and 24.04) and varying hardware configurations.

## System Architecture
The environment utilizes a dual-layer workspace approach:
* [cite_start]**Underlay (`/opt/xarm_ws`):** Contains the core `xarm_ros2` drivers and MoveIt 2 dependencies, pre-built during the Docker image creation[cite: 3, 4, 5].
* **Overlay (`/ros2_ws`):** Your local workspace mapped from the host machine, used for custom packages and project-specific code.

---

## Prerequisites

### 1. Docker & Docker Compose
Ensure Docker and the Docker Compose plugin are installed on your host machine.

### 2. Hardware-Specific Drivers
Depending on your GPU, additional setup may be required:

| Hardware | Requirement |
| :--- | :--- |
| **NVIDIA GPU** | Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html). |
| **AMD/Intel GPU** | No additional toolkit needed; ensure `mesa-utils` are on the host. |
| **No GPU (Headless/Lab)** | No hardware requirements; the system will fallback to software rendering. |

---

## Getting Started

### 1. Clone the Repository
```bash
git clone [https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git)
cd YOUR_REPO_NAME

2. Build the Environment

This command builds the ROS 2 Humble image and compiles the xArm drivers. This only needs to be run once or when the Dockerfile is modified.
Bash

sudo docker compose build

3. Launch the Container

Use the provided automation script. This script detects your hardware and configures OpenGL rendering automatically.

Bash

chmod +x humble-go.sh
./humble-go.sh

Configuration Details
GPU Modes

The humble-go.sh script automatically manages the LIBGL_ALWAYS_SOFTWARE environment variable:

    NVIDIA: Uses hardware acceleration via the NVIDIA Container Toolkit.

    General GPU (AMD/Intel): Uses host-side DRI device mapping.

    No GPU: Sets LIBGL_ALWAYS_SOFTWARE=1 to ensure RViz and Gazebo stability on lab machines without dedicated graphics.

Workspace Management

Your custom code should be placed in workspaces/<project_name>/src.
Inside the container, navigate to your project folder to build:
Bash

cd /ros2_ws/workspaces/project_1
colcon build --symlink-install
source install/setup.bash

Included Dependencies

The Docker image includes the following pre-installed stacks:

    ROS 2 Humble Desktop Full 

    MoveIt 2 

    Gazebo (Classic and ROS 2 Integration) 

    xArm ROS 2 Suite (Source build) 

    ros2_control & ros2_controllers 


---

### Final Next Step
Now that your environment is fully documented and configured, **would you like me to walk you through the commands to push this entire project to your GitHub repository?**