# Use the official ROS 2 Humble desktop image
FROM osrf/ros:humble-desktop-full

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install basic development tools
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    lsb-release \
    wget \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# 1. Install MoveIt 2
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Gazebo
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gz \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# 3. Install xArm hardware/control dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-control-msgs \
    ros-humble-realtime-tools \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# 4. Pre-build the xArm repo in a global "Underlay" workspace
WORKDIR /opt/xarm_ws/src
RUN git clone https://github.com/xArm-Developer/xarm_ros2.git . && \
    git submodule update --init --recursive

WORKDIR /opt/xarm_ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Set up the user workspace
WORKDIR /ros2_ws

# Automatically source ROS and xArm environments in every new bash session
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /opt/xarm_ws/install/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash"]