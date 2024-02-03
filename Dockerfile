ARG ROS_DISTRO=rolling
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get update \
    && apt-get install -y \
        ros-dev-tools \
        wget
        
WORKDIR /root/ros2_ws/
RUN mkdir -p src
COPY tools/ros2_dependencies.repos .
RUN vcs import --input ros2_dependencies.repos src
RUN rosdep init

COPY . src/grid_map
RUN ls src/grid_map

SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && rosdep update

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install -y --ignore-src --from-paths src --skip-keys slam_toolbox

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \ 
    && colcon build --symlink-install --packages-up-to grid_map

