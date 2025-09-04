# syntax=docker/dockerfile:1

# Duckietown base with ROS and catkin tools (multi-arch manifest)
FROM duckietown/dt-ros-commons:daffy

SHELL ["/bin/bash", "-lc"]

ENV ROS_WORKSPACE=/code/catkin_ws
WORKDIR ${ROS_WORKSPACE}

# Create workspace
RUN mkdir -p ${ROS_WORKSPACE}/src

# Copy ROS packages from this project
COPY packages/ ${ROS_WORKSPACE}/src/

# Copy launch entrypoint script (project-level)
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

# Build the catkin workspace
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && cd ${ROS_WORKSPACE} \
    && catkin build --no-status

# Default command: source overlays and delegate to project launcher
CMD ["bash", "-lc", "source /opt/ros/$ROS_DISTRO/setup.bash && source ${ROS_WORKSPACE}/devel/setup.bash && /launch.sh"]
