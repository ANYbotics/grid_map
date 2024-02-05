ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-core

WORKDIR /root/ros2_ws/

# Install essential dependencies
RUN apt-get update && \
    apt-get install -y \
        ros-dev-tools \
        wget && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Clone dependencies
COPY tools/ros2_dependencies.repos .
RUN mkdir -p src && \
    vcs import --input ros2_dependencies.repos src

# Initialize rosdep
RUN rosdep init

# Copy source code
COPY . src/grid_map

# Install dependencies
SHELL ["/bin/bash", "-c"]
RUN apt-get update && \
    rosdep update && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -y --ignore-src --from-paths src --skip-keys slam_toolbox --skip-keys gazebo_ros_pkgs --skip-keys turtlebot3_gazebo && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --packages-up-to grid_map
