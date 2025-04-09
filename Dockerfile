FROM registry.screamtrumpet.csie.ncku.edu.tw/pros_images/pros_base_image:latest
ENV ROS2_WS=/workspaces
ENV WS_MOVEIT=/root/ws_moveit
ENV ROS_DOMAIN_ID=1
ENV ROS_DISTRO=humble
ARG THREADS=4
ARG TARGETPLATFORM

SHELL ["/bin/bash", "-c"]

##### System Upgrade #####
RUN apt update && \
    apt upgrade -y && \
    apt autoremove -y && \
    apt autoclean -y && \
    ulimit -s 65536 && \
    pip3 install --no-cache-dir --upgrade pip

##### colcon Installation #####
# Prepare Source Code
RUN mkdir -p ${WS_MOVEIT}/src
WORKDIR ${WS_MOVEIT}/src
RUN git clone -b humble https://github.com/moveit/moveit2_tutorials && \
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
WORKDIR ${WS_MOVEIT}
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    # # Limit the number of threads to 1 for arm64 architecture
    # # to prevent cpp segmentation fault in qemu.
    # if [ "$TARGETPLATFORM" = "linux/arm64" ]; then \
    #     MAKEFLAGS=-j1 colcon build --mixin release --parallel-workers 1; \
    # else \
    #     colcon build --mixin release; \
    # fi && \
    colcon build --mixin release && \
    echo "source ${WS_MOVEIT}/install/setup.bash" >> /root/.bashrc

##### Post-Settings #####
WORKDIR ${ROS2_WS}

# Update entrypoint
COPY ./ros_entrypoint.bash /ros_entrypoint.bash
RUN chmod +x /ros_entrypoint.bash && \

# Clear tmp and cache
    rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]
