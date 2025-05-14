FROM ros:iron-ros-base

# Install ROS2 Nav2 and common tools
RUN apt-get update && apt-get install -y \
    ros-iron-navigation2 \
    ros-iron-nav2-bringup \
    ros-iron-rviz2 \
    ros-iron-tf2-tools \
    python3-colcon-common-extensions \
    git curl wget vim less \
  && rm -rf /var/lib/apt/lists/*

# Build arguments with defaults
ARG USER_ID=1000
ARG GROUP_ID=1000

# Create group and user matching host UID/GID
RUN groupadd -g 1000 rosgroup && useradd -m -u 1000 -g rosgroup rosuser


# Copy entrypoint and set permissions (as root)
COPY ../ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Switch to non-root user
USER rosuser
WORKDIR /home/rosuser

# Create ROS2 workspace directory
RUN mkdir -p /home/rosuser/nav2_ws/src
WORKDIR /home/rosuser/nav2_ws
ENV ROS_WS=/home/rosuser/nav2_ws

# Set entrypoint to source ROS and workspace
ENTRYPOINT ["/ros_entrypoint.sh"]
