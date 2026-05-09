#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia MoveIt/RViz no Docker + bridge no Nano
#
# Garante estado DDS limpo: reinicia bridge no Nano via setsid
# (imune a SIGHUP do SSH), limpa processos antigos no Docker,
# depois lança MoveIt.
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
  pkill -9 -f "ros2 launch mycobot_hw_interface" 2>/dev/null || true
  sleep 2
  truncate -s 0 /tmp/bridge.log
  setsid bash -c "bash ~/start_bridge.sh > /tmp/bridge.log 2>&1" < /dev/null > /dev/null 2>&1 &
  disown -a
'
sleep 6
BRIDGE_OK=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} 'ps aux | grep -v grep | grep -c mycobot_bridge')
if [ "$BRIDGE_OK" -gt 0 ]; then
  echo "  Bridge OK (mycobot_bridge running on Nano)"
else
  echo "  Bridge FAILED — check via: sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/bridge.log'"
  exit 1
fi

echo ""
echo "========================================"
echo "  [2/3] Limpando processos ros2 no Docker"
echo "========================================"
docker exec mycobot_ros2 bash -c "
  ps aux | grep -E 'ros2|rviz|move_group|joint_state|robot_state|static_transform' | grep -v grep | awk '{print \$2}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
" 2>/dev/null
echo "  Docker limpo"

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
