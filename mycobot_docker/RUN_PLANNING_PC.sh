#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia o MoveIt/RViz no Docker
# CycloneDDS com peer estático para o Nano (sem Discovery Server)
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"

echo "========================================"
echo "  Ligando MoveIt 2 + RViz no PC (DDS)"
echo "========================================"

xhost +local:root 2>/dev/null

docker exec mycobot_ros2 pkill -9 -f ros2 2>/dev/null || true

docker exec -it mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"
