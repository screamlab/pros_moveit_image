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
    apt update && \

# Bootstrap rosdep and setup colcon mixin and metadata ###
    rosdep update --rosdistro $ROS_DISTRO && \
    colcon mixin update && \
    colcon metadata update && \

# Install the system dependencies for all ROS packages located in the `src` directory.
    rosdep install -q -y -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO

### Moveit2 Installation ###
RUN cd ${WS_MOVEIT} && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --mixin release --packages-select moveit_resources_panda_description \
                                                   moveit_common \
                                                   moveit_resources_pr2_description \
                                                   srdfdom \
                                                   moveit_resources_panda_moveit_config \
                                                   moveit_core \
                                                   launch_param_builder \
                                                   moveit_configs_utils \
                                                   moveit_ros_occupancy_map_monitor \
                                                   moveit_ros_planning
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release --packages-select moveit_resources_fanuc_description \
                                                   moveit_resources_fanuc_moveit_config \
                                                   moveit_kinematics \
                                                   moveit_simple_controller_manager \
                                                   moveit_ros_move_group \
                                                   moveit_ros_warehouse \
                                                   moveit_planners_ompl \
                                                   moveit_ros_planning_interface \
                                                   moveit_resources_prbt_support \
                                                   moveit_resources_prbt_ikfast_manipulator_plugin
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release --packages-select moveit_ros_robot_interaction \
                                                   chomp_motion_planner \
                                                   moveit_resources_prbt_moveit_config \
                                                   moveit_ros_visualization \
                                                   pilz_industrial_motion_planner_testutils \
                                                   moveit_resources_prbt_pg70_support \
                                                   moveit_planners_chomp \
                                                   pilz_industrial_motion_planner \
                                                   moveit_setup_framework \
                                                   moveit_planners
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release --packages-select moveit_task_constructor_msgs \
                                                   rviz_marker_tools \
                                                   moveit_task_constructor_core \
                                                   moveit_plugins \
                                                   moveit_ros_benchmarks \
                                                   moveit_setup_app_plugins \
                                                   moveit_setup_controllers \
                                                   moveit_setup_core_plugins \
                                                   moveit_setup_srdf_plugins \
                                                   moveit_ros_perception
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release --packages-select moveit_ros \
                                                   moveit_setup_assistant \
                                                   moveit_visual_tools \
                                                   moveit_hybrid_planning \
                                                   moveit_servo \
                                                   moveit \
                                                   moveit_task_constructor_capabilities \
                                                   rosparam_shortcuts \
                                                   moveit_resources \
                                                   moveit_chomp_optimizer_adapter
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release --packages-select moveit_ros_control_interface \
                                                   moveit_runtime \
                                                   moveit_task_constructor_visualization \
                                                   moveit2_tutorials \
                                                   moveit_task_constructor_demo
RUN source ${WS_MOVEIT}/install/setup.bash && \
    colcon build --mixin release && \
    echo "source ${WS_MOVEIT}/install/setup.bash" >> /root/.bashrc

##### Post-Settings #####
# Clear tmp and cache
RUN rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]
