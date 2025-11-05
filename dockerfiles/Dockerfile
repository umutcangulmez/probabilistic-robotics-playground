# ROS 2 Humble with Gazebo Fortress
FROM osrf/ros:humble-desktop-full

# Install Gazebo Fortress + ROS 2 bridges and tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        gazebo \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros2-control \
        ros-humble-rviz2 \
        ros-humble-turtlebot3* \
        python3-colcon-common-extensions \
        git && \
    rm -rf /var/lib/apt/lists/*


RUN apt-get update
RUN apt-get install -y gdb vim
RUN apt-get install -y mlocate
RUN updatedb

# Environment setup
ENV TURTLEBOT3_MODEL=burger
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:/opt/ros/humble/share/turtlebot3_gazebo/models"



# Default work directory
WORKDIR /root/ros2_ws
RUN mkdir -p src

# Source ROS2 automatically
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENTRYPOINT ["bash", "-c", "source /opt/ros/humble/setup.bash && exec bash"]

