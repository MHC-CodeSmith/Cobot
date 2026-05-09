FROM osrf/ros:galactic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    git \
    wget \
    curl \
    sshpass \
    ros-galactic-joint-state-publisher-gui \
    ros-galactic-robot-state-publisher \
    ros-galactic-rviz2 \
    ros-galactic-xacro \
    ros-galactic-moveit \
    ros-galactic-moveit-setup-assistant \
    ros-galactic-backward-ros \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pymycobot --upgrade

# custom_ws é montado como volume em runtime — o build é feito lá dentro
RUN mkdir -p /root/custom_ws/src

CMD ["bash"]
