#!/bin/bash
# ============================================================
# RUN_LAPTOP_3D.sh — Webcam do notebook analisa o braço do robô
# ============================================================
# TODO (Fase futura):
#   Webcam do notebook (PC) vê o braço do robô do exterior.
#   Pipeline de visão estima posição 3D do pulso/mão via:
#     - MediaPipe Hands / Pose monocular + heurísticas de profundidade
#     - OU câmera estéreo / RGBD
#   arm_mapper_node converte posição 3D da mão → pose target no workspace do robô
#   target_follower_node usa MoveIt plan+execute para seguir o pulso
#
# Por ora este script apenas inicia o vision_node com webcam local
# e mostra os landmarks do braço (sem movimento do robô).
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"

echo "========================================"
echo "  Laptop 3D — webcam analisa o braco"
echo "  (modo visualizacao — sem movimento)"
echo "========================================"

docker exec mycobot_ros2 pkill -9 -f vision_node   2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f face_follower 2>/dev/null || true
sleep 1

docker exec -d mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py \
    use_arm_camera:=false \
    2>&1 | tee /tmp/teleop.log
"

echo ""
echo "  Vision node rodando com webcam local"
echo "  Ver landmarks:"
echo "    docker exec -it mycobot_ros2 bash -c \\"
echo "      'source /opt/ros/galactic/setup.bash && source /root/custom_ws/install/setup.bash && \\"
echo "       export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=$CYCLONE_XML && \\"
echo "       ros2 run image_tools showimage --ros-args -r image:=/human/image_debug'"
echo ""
echo "  (movimento do braco via tracking 3D: nao implementado ainda)"
