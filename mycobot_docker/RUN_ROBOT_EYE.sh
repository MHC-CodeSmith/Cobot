#!/bin/bash
# ============================================================
# RUN_ROBOT_EYE.sh — O robô vê você e te segue
# ============================================================
# Câmera do braço (Nano /dev/video0) → visão com MediaPipe →
# face_follower move joint1/joint2 para centralizar o rosto.
# O RViz mostra o que o robô está vendo (painel "Olho do Robo").
# O overlay mostra o alvo (retangulo verde = rosto centrado).
#
# PRÉ-REQUISITO: RUN_PLANNING_PC.sh rodando
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"
NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"

echo "========================================"
echo "  Robot Eye — o robo ve voce"
echo "  Camera: braco do robo (Nano)"
echo "========================================"

# ── [1/4] Câmera no Nano ─────────────────────────────────────
echo ""
echo "[1/4] Iniciando câmera do braço no Nano..."

timeout 8 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 \
  ${NANO_USER}@${NANO_IP} \
  'pkill -9 -f arm_camera_node 2>/dev/null; true' 2>/dev/null || true
sleep 1

timeout 10 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
  ${NANO_USER}@${NANO_IP} \
  'bash -c "
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=\$HOME/cyclonedds.xml
    source /opt/ros/galactic/setup.bash
    source \$HOME/custom_ws/install/setup.bash
    nohup ros2 run mycobot_hw_interface arm_camera_node \
      >/tmp/arm_camera.log 2>&1 </dev/null &
  " </dev/null >/dev/null 2>/dev/null' 2>/dev/null || true

sleep 3

CAM_OK=$(timeout 6 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  ${NANO_USER}@${NANO_IP} \
  'ps aux | grep -v grep | grep -c arm_camera_node' 2>/dev/null || echo 0)

if [ "${CAM_OK:-0}" -gt 0 ]; then
  echo "  Camera OK — /arm_camera/image_raw publicando"
else
  echo "  AVISO: camera nao detectada no Nano"
  echo "  Verifique: sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/arm_camera.log'"
fi

# ── [2/4] Vision + face follower no Docker ───────────────────
echo ""
echo "[2/4] Iniciando vision_node + face_follower no Docker..."

docker exec mycobot_ros2 pkill -9 -f vision_node   2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f face_follower 2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f showimage     2>/dev/null || true
sleep 1

docker exec -d mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py \
    use_arm_camera:=true \
    2>&1 | tee /tmp/teleop.log
"

# ── [3/4] Habilita movimento ─────────────────────────────────
echo ""
echo "[3/4] Aguardando nós subirem e habilitando movimento..."
sleep 5

docker exec mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool 'data: true'
" 2>/dev/null && echo "  Movimento HABILITADO" \
             || echo "  (habilite manualmente quando os nos subirem)"

# ── [4/4] Abre janela com o que o robô vê ────────────────────
echo ""
echo "[4/4] Abrindo câmera do robô..."

if [ -n "$DISPLAY" ]; then
  docker exec -d mycobot_ros2 bash -c "
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=$CYCLONE_XML
    export DISPLAY=$DISPLAY
    source /opt/ros/galactic/setup.bash
    source /root/custom_ws/install/setup.bash
    ros2 run image_tools showimage --ros-args -r image:=/human/image_debug \
      2>&1 | tee /tmp/showimage.log
  " 2>/dev/null && echo "  Janela de camera aberta (overlay com alvo visivel)"
else
  echo "  DISPLAY nao configurado — veja o painel 'Olho do Robo' no RViz"
fi

echo ""
echo "========================================"
echo "  PRONTO — Robo esta te seguindo"
echo "========================================"
echo ""
echo "  Overlay na camera:"
echo "    - Retangulo VERDE  = rosto centrado na janela alvo"
echo "    - Retangulo LARANJA = robo corrigindo posicao"
echo "    - Ponto = nariz detectado"
echo ""
echo "  Log em tempo real:"
echo "    docker exec mycobot_ros2 tail -f /tmp/teleop.log"
echo ""
echo "  Parar tudo:"
echo "    docker exec mycobot_ros2 pkill -9 -f vision_node"
echo "    sshpass -p Elephant ssh er@192.168.0.250 'pkill -9 -f arm_camera_node'"
