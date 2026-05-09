#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia o MoveIt/RViz no Docker
# CycloneDDS com peer estático para o Nano (sem Discovery Server)
#
# Antes de subir o MoveIt, REINICIA o bridge no Nano para garantir
# estado DDS limpo (CycloneDDS pode reter subscribers mortos e
# bloquear entrega de dados a novos subscribers).
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"
NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"

echo "========================================"
echo "  [1/3] Reiniciando bridge no Nano"
echo "========================================"
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} '
  pkill -9 -f mycobot_bridge 2>/dev/null || true
  sleep 1
  nohup bash ~/start_bridge.sh > /tmp/bridge.log 2>&1 < /dev/null &
  disown
  sleep 4
  pgrep -a -f mycobot_bridge > /dev/null && echo "  Bridge OK" || echo "  Bridge FAILED"
'

echo ""
echo "========================================"
echo "  [2/3] Limpando processos ros2 no Docker"
echo "========================================"
docker exec mycobot_ros2 bash -c "
  ps aux | grep -E 'ros2|rviz|move_group|joint_state|robot_state|static_transform' | grep -v grep | awk '{print \$2}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
" 2>/dev/null

xhost +local:root 2>/dev/null

echo ""
echo "========================================"
echo "  [3/3] Lançando MoveIt 2 + RViz"
echo "========================================"
docker exec -it mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"
