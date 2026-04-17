FROM osrf/ros:galactic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    python3-vcstool \
    sudo \
    ros-galactic-joint-state-publisher-gui \
    ros-galactic-robot-state-publisher \
    ros-galactic-rviz2 \
    ros-galactic-xacro \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-moveit \
    ros-galactic-backward-ros \
    ros-galactic-moveit-setup-assistant \
    && rm -rf /var/lib/apt/lists/*

# Install pymycobot
RUN pip3 install pymycobot --upgrade

# Setup ROS workspace
RUN mkdir -p /root/colcon_ws/src
WORKDIR /root/colcon_ws/src

# Clone the repository
RUN git clone -b galactic --depth 1 https://github.com/elephantrobotics/mycobot_ros2.git

WORKDIR /root/colcon_ws

SHELL ["/bin/bash", "-c"]

# Build
RUN source /opt/ros/galactic/setup.bash && colcon build

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc && \
    echo "source /root/colcon_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]
