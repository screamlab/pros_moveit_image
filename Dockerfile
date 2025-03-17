FROM registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:latest
ENV ROS2_WS /workspaces
ENV WS_MOVEIT /root/ws_moveit
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO=humble
ARG THREADS=4
ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"]

##### Environment Settings #####
WORKDIR ${ROS2_WS}

# System Upgrade
RUN apt update && \
    apt upgrade -y && \
    apt autoremove -y && \
    apt autoclean -y && \

    pip3 install --no-cache-dir --upgrade pip

##### colcon Installation #####
# Prepare Source Code
RUN mkdir -p ${WS_MOVEIT}/src && \
    cd ${WS_MOVEIT}/src && \
    git clone -b humble https://github.com/moveit/moveit2_tutorials && \
    vcs import --recursive < moveit2_tutorials/moveit2_tutorials.repos && \
    apt remove ros-$ROS_DISTRO-moveit* -y && \
    apt update

# Bootstrap rosdep and setup colcon mixin and metadata ###
RUN rosdep update --rosdistro $ROS_DISTRO && \
    colcon mixin update && \
    colcon metadata update

# Install the system dependencies for all ROS packages located in the `src` directory.
RUN rosdep install -q -y -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO

### Moveit2 Installation ###
RUN cd ${WS_MOVEIT} && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --mixin release && \
    echo "source ${WS_MOVEIT}/install/setup.bash" >> /root/.bashrc

##### Post-Settings #####
# Clear tmp and cache
RUN rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]
