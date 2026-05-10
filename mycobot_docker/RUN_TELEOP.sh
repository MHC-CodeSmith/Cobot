#!/bin/bash
# ============================================================
# RUN_TELEOP.sh — Adiciona vision teleop por cima do sistema existente
#
# PRÉ-REQUISITO: RUN_PLANNING_PC.sh deve estar rodando
# (MoveIt + RViz + bridge no Nano já ativos)
#
# USA CycloneDDS (igual ao RUN_PLANNING_PC.sh) para que
# vision_node e MoveIt se enxerguem no mesmo domínio DDS.
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"

echo "========================================"
echo "  Vision Teleop — myCobot 280 JN"
echo "  RMW: CycloneDDS | Domain: 42"
echo "========================================"

# Garante que não há visão node zumbi (pode ter câmera travada)
docker exec mycobot_ros2 pkill -9 -f vision_node 2>/dev/null || true
sleep 1

# Roda o teleop em background dentro do container
docker exec -d mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py \
    2>&1 | tee /tmp/teleop.log
"

echo ""
echo "=== Teleop iniciado em background ==="
echo ""
echo "  Ver log em tempo real:"
echo "    docker exec mycobot_ros2 tail -f /tmp/teleop.log"
echo ""
echo "  Verificar tópicos:"
echo "    docker exec mycobot_ros2 bash -c \\"
echo "      'export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \\"
echo "       export CYCLONEDDS_URI=$CYCLONE_XML && \\"
echo "       source /opt/ros/galactic/setup.bash && \\"
echo "       ros2 topic list | grep human'"
echo ""
echo "  Parar teleop:"
echo "    docker exec mycobot_ros2 pkill -9 -f vision_node"
