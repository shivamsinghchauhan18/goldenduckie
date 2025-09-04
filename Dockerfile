# syntax=docker/dockerfile:1

# Base image with ROS and catkin tools (Duckietown daffy / ROS Noetic)
FROM duckietown/dt-ros-commons:daffy

SHELL ["/bin/bash", "-lc"]

ENV ROS_WORKSPACE=/code/catkin_ws
WORKDIR ${ROS_WORKSPACE}

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_APT_MIRROR="http://packages.ros.org/ros/ubuntu"
ARG ROS_APT_MIRROR_ALT="http://mirror.umd.edu/ros/ubuntu"
## System dependencies + refresh ROS keyring to avoid GPG errors; add retries and IPv4
RUN set -eux; \
    echo 'Acquire::Retries "5";\nAcquire::http::Timeout "30";\nAcquire::ForceIPv4 "true";' > /etc/apt/apt.conf.d/99retries; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        curl \
        gnupg2; \
    mkdir -p /usr/share/keyrings; \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg; 
RUN set -eux; \
    UB_CODENAME=$(. /etc/os-release && echo $UBUNTU_CODENAME); \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_MIRROR} ${UB_CODENAME} main" > /etc/apt/sources.list.d/ros1-latest.list; \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] ${ROS_APT_MIRROR_ALT} ${UB_CODENAME} main" > /etc/apt/sources.list.d/ros1-latest-alt.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        ros-$ROS_DISTRO-image-geometry \
        libyaml-cpp-dev; \
    rm -rf /var/lib/apt/lists/*



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
