# syntax=docker/dockerfile:1

# Base image with ROS and catkin tools (Duckietown daffy / ROS Noetic)
FROM duckietown/dt-ros-commons:daffy

SHELL ["/bin/bash", "-lc"]

ENV ROS_WORKSPACE=/code/catkin_ws
WORKDIR ${ROS_WORKSPACE}

# System dependencies required by packages in this repo
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-$ROS_DISTRO-image-geometry \
        libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace structure
RUN mkdir -p ${ROS_WORKSPACE}/src

# Copy all local packages (build the whole project)
COPY dt-core/ ${ROS_WORKSPACE}/src/dt-core/
COPY dt-ros-commons/ ${ROS_WORKSPACE}/src/dt-ros-commons/
COPY pp-navigation/ ${ROS_WORKSPACE}/src/pp-navigation/

# Ensure launch script is executable
RUN chmod +x ${ROS_WORKSPACE}/src/pp-navigation/launch.sh || true

# Build the catkin workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ${ROS_WORKSPACE} \
    && catkin build --no-status

# Default command: source overlays and run the navigation launch script
CMD ["bash", "-lc", "source /opt/ros/$ROS_DISTRO/setup.bash && source ${ROS_WORKSPACE}/devel/setup.bash && ${ROS_WORKSPACE}/src/pp-navigation/launch.sh"]
