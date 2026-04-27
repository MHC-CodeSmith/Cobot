#!/bin/bash
# Limpeza
echo "Limpando processos antigos..."
pkill -9 -f mycobot
pkill -9 -f ros2
sleep 2

# Ambiente
source /opt/ros/galactic/setup.bash
source ~/custom_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Rodar o bridge
echo "Iniciando Bridge MyCobot..."
ros2 launch mycobot_hw_interface mycobot_hw.launch.py mock:=False baud:=1000000
