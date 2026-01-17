# ROS 2 Humble base
FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

# Install Gazebo Fortress (Ignition Gazebo) + ROS 2 bridges
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gz-fortress \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-image \
        ros-humble-foxglove-bridge \
        ros-humble-rviz2 \
        ros-humble-rosbridge-server  \
        ros-humble-turtlebot3* \
        python3-colcon-common-extensions \
        git \
        gdb \
        vim \
        mlocate && \
    rm -rf /var/lib/apt/lists/*


RUN apt-get install -y mlocate
RUN updatedb

# Environment setup
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Gazebo Fortress environment variables
ENV GZ_VERSION=fortress
ENV GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
ENV GZ_SIM_RESOURCE_PATH=/opt/ros/humble/share


# Default work directory
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Source ROS2 automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

