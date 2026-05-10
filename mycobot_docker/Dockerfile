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

# Vision teleop dependencies
# mediapipe 0.10.5 = última versão compatível com Python 3.8 (Ubuntu 20.04 / Galactic)
# cv_bridge via apt para integração com ROS 2
RUN apt-get update && apt-get install -y ros-galactic-cv-bridge && rm -rf /var/lib/apt/lists/*
RUN pip3 install "mediapipe==0.10.5" numpy opencv-python

# custom_ws é montado como volume em runtime — o build é feito lá dentro
RUN mkdir -p /root/custom_ws/src

# XDG_RUNTIME_DIR necessário para Qt/RViz2 (evita SIGSEGV no moveit_rviz_plugin)
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

CMD ["bash"]
