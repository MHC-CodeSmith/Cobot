#!/bin/bash
export PYTHONUNBUFFERED=1
source /opt/ros/galactic/setup.bash
source /home/er/custom_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# Limpar processos antigos (pkill local)
pkill -9 -f mycobot_bridge || true
pkill -9 -f udp_bridge_nano || true

# Iniciar Bridge de Hardware em background
ros2 launch mycobot_hw_interface mycobot_hw.launch.py mock:=False baud:=1000000 > /home/er/hardware.log 2>&1 &

# Iniciar Tunel UDP
python3 /home/er/udp_bridge_nano.py > /home/er/udp.log 2>&1 &

# Manter vivo
wait
