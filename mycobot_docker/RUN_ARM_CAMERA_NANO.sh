#!/bin/bash
# ============================================================
# RUN_ARM_CAMERA_NANO.sh
# Inicia o nó de câmera do braço no Nano via SSH.
# Publica /arm_camera/image_raw → PC processa com MediaPipe.
#
# PRÉ-REQUISITO: RUN_PLANNING_PC.sh rodando (bridge + MoveIt ativos)
#
# Depois desse script, rode no PC:
#   ./mycobot_docker/RUN_TELEOP.sh use_arm_camera
# ============================================================

NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"

echo "========================================"
echo "  Câmera do Braço — Nano → PC"
echo "  Tópico: /arm_camera/image_raw"
echo "========================================"

# Mata câmera antiga
echo "[1/2] Matando camera antiga no Nano..."
timeout 8 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 \
  ${NANO_USER}@${NANO_IP} \
  'pkill -9 -f arm_camera_node 2>/dev/null; true' 2>/dev/null || true

sleep 1

# Inicia câmera em background
echo "[2/2] Iniciando arm_camera_node no Nano..."
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

# Verifica
CAM_OK=$(timeout 6 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  ${NANO_USER}@${NANO_IP} \
  'ps aux | grep -v grep | grep -c arm_camera_node' 2>/dev/null || echo 0)

if [ "${CAM_OK:-0}" -gt 0 ]; then
  echo ""
  echo "  Câmera OK — /arm_camera/image_raw publicando"
  echo ""
  echo "  Agora rode no PC:"
  echo "    ./mycobot_docker/RUN_TELEOP.sh arm"
  echo ""
  echo "  Ver log câmera:"
  echo "    sshpass -p Elephant ssh er@192.168.0.250 'tail -f /tmp/arm_camera.log'"
else
  echo ""
  echo "  FALHOU — verifique:"
  echo "    sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/arm_camera.log'"
fi
