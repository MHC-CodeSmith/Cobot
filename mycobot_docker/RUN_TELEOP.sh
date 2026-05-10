#!/bin/bash
# ============================================================
# RUN_TELEOP.sh — Vision teleop por cima do MoveIt existente
#
# PRÉ-REQUISITO: RUN_PLANNING_PC.sh rodando (MoveIt + bridge ativos)
#
# Modos:
#   ./RUN_TELEOP.sh          → câmera local do PC (webcam)
#   ./RUN_TELEOP.sh arm      → câmera do braço no Nano
#                              (requer RUN_ARM_CAMERA_NANO.sh primeiro)
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"
MODE="${1:-local}"

if [ "$MODE" = "arm" ]; then
  USE_ARM_CAM="true"
  CAM_DESC="câmera do BRAÇO (Nano → PC)"
else
  USE_ARM_CAM="false"
  CAM_DESC="câmera LOCAL (webcam PC)"
fi

echo "========================================"
echo "  Vision Teleop — myCobot 280 JN"
echo "  Modo: $CAM_DESC"
echo "  RMW: CycloneDDS | Domain: 42"
echo "========================================"

# Mata visão anterior
docker exec mycobot_ros2 pkill -9 -f vision_node    2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f face_follower  2>/dev/null || true
sleep 1

# Inicia em background
docker exec -d mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py \
    use_arm_camera:=$USE_ARM_CAM \
    2>&1 | tee /tmp/teleop.log
"

echo ""
echo "=== Teleop iniciado ($CAM_DESC) ==="
echo ""
echo "  Acompanhar log:"
echo "    docker exec mycobot_ros2 tail -f /tmp/teleop.log"
echo ""
echo "  Ver landmarks no rosto:"
echo "    docker exec -it mycobot_ros2 bash -c \\"
echo "      'source /opt/ros/galactic/setup.bash && source /root/custom_ws/install/setup.bash && \\"
echo "       export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \\"
echo "       export CYCLONEDDS_URI=$CYCLONE_XML && \\"
echo "       ros2 run image_tools showimage --ros-args -r image:=/human/image_debug'"
echo ""
echo "  Habilitar movimento (após log mostrar 'FaceFollower pronto'):"
echo "    docker exec mycobot_ros2 bash -c \\"
echo "      'source /opt/ros/galactic/setup.bash && source /root/custom_ws/install/setup.bash && \\"
echo "       export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \\"
echo "       export CYCLONEDDS_URI=$CYCLONE_XML && \\"
echo "       ros2 topic pub --once /face_follower/enabled std_msgs/Bool \"data: true\"'"
echo ""
echo "  Parar:"
echo "    docker exec mycobot_ros2 pkill -9 -f vision_node"
