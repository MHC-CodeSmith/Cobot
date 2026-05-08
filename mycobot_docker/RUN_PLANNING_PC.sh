#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia o MoveIt/RViz no Docker
# ============================================================
# O NANO (Cobot) é o servidor de descoberta
DISCOVERY_IP="192.168.0.250"
DISCOVERY_PORT="11811"
XML_PATH="/root/custom_ws/fastdds_super_client.xml"

echo "========================================"
echo "  Ligando MoveIt 2 + RViz no PC (DDS)"
echo "========================================"

# Garante acesso ao X11
xhost +local:root 2>/dev/null

# Limpa processos antigos no Docker
docker exec mycobot_ros2 pkill -9 -f ros2 || true

# Roda o MoveIt com Discovery Server e Super Client
docker exec -it mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DISCOVERY_SERVER=$DISCOVERY_IP:$DISCOVERY_PORT
  export FASTRTPS_DEFAULT_PROFILES_FILE=$XML_PATH
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"
