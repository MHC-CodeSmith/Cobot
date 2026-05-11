#!/bin/bash
# ============================================================
# RUN_VISUAL_SERVO.sh — Visual servoing eye-in-hand
#
# Ctrl+C limpa TUDO: Docker nodes + câmera + bridge no Nano.
# Se RUN_PLANNING_PC.sh já estiver a correr, reutiliza o bridge
# existente sem o matar (não interfere com o planning).
# ============================================================

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"
NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"
BRIDGE_STARTED_HERE=0   # 1 se este script arrancou o bridge

# ── Cleanup — corre ao Ctrl+C ou exit ────────────────────────────────
cleanup() {
  echo ""
  echo "A parar visual servo..."

  # Docker: mata nós de visão e showimage
  docker exec mycobot_ros2 pkill -9 -f face_detector           2>/dev/null || true
  docker exec mycobot_ros2 pkill -9 -f visual_servo_controller 2>/dev/null || true
  docker exec mycobot_ros2 pkill -9 -f joint_state_relay        2>/dev/null || true
  docker exec mycobot_ros2 pkill -9 -f static_transform         2>/dev/null || true
  docker exec mycobot_ros2 pkill -9 -f showimage                2>/dev/null || true

  # Nano: câmera sempre
  sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    ${NANO_USER}@${NANO_IP} \
    'pkill -9 -f arm_camera_node 2>/dev/null; true' 2>/dev/null || true

  # Nano: bridge só se foi arrancado aqui (não matar o do planning)
  if [ "$BRIDGE_STARTED_HERE" -eq 1 ]; then
    sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
      ${NANO_USER}@${NANO_IP} \
      'pkill -9 -f mycobot_bridge 2>/dev/null; true' 2>/dev/null || true
    echo "  Bridge parado."
  else
    echo "  Bridge mantido (pertence ao RUN_PLANNING_PC.sh)."
  fi

  echo "Limpo."
  exit 0
}
trap cleanup SIGINT SIGTERM EXIT

echo "========================================"
echo "  Visual Servo — eye-in-hand tracking"
echo "========================================"

# ── [1/5] Rebuild no Docker ──────────────────────────────────────────
echo ""
echo "[1/5] Rebuild mycobot_vision_teleop..."
docker exec mycobot_ros2 bash -c "
  source /opt/ros/galactic/setup.bash
  cd /root/custom_ws
  colcon build --symlink-install \
    --packages-select mycobot_vision_teleop \
    2>&1 | tail -4
" && echo "  Docker: build OK" || echo "  Docker: build falhou (usando versão anterior)"

# ── [2/5] Bridge — reutiliza se já existir ───────────────────────────
echo ""
echo "[2/5] Verificando bridge no Nano..."

BRIDGE_RUNNING=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 \
  ${NANO_USER}@${NANO_IP} \
  'ps aux | grep -v grep | grep -c mycobot_bridge' 2>/dev/null || echo 0)

if [ "${BRIDGE_RUNNING:-0}" -gt 0 ]; then
  echo "  Bridge já a correr (RUN_PLANNING_PC.sh ativo) — reutilizando."
  BRIDGE_STARTED_HERE=0
else
  echo "  Bridge não encontrado — arrancando..."
  timeout 12 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=10 \
    ${NANO_USER}@${NANO_IP} \
    'bash -c "
      export ROS_DOMAIN_ID=42
      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
      export CYCLONEDDS_URI=\$HOME/cyclonedds.xml
      source /opt/ros/galactic/setup.bash
      source \$HOME/custom_ws/install/setup.bash
      nohup ros2 launch mycobot_hw_interface mycobot_hw.launch.py \
        port:=/dev/ttyTHS1 baud:=1000000 mock:=False \
        >/tmp/bridge.log 2>&1 </dev/null &
    " </dev/null >/dev/null 2>/dev/null' 2>/dev/null || true

  sleep 4
  BRIDGE_OK=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
    ${NANO_USER}@${NANO_IP} \
    'ps aux | grep -v grep | grep -c mycobot_bridge' 2>/dev/null || echo 0)

  if [ "${BRIDGE_OK:-0}" -gt 0 ]; then
    echo "  Bridge OK"
    BRIDGE_STARTED_HERE=1
  else
    echo "  AVISO: bridge não arrancou — sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/bridge.log'"
  fi
fi

# ── [3/5] Câmera no Nano ─────────────────────────────────────────────
echo ""
echo "[3/5] Câmera do braço (320×240@20fps)..."

sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 \
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
      --ros-args -p width:=320 -p height:=240 -p fps:=20 \
      >/tmp/arm_camera.log 2>&1 </dev/null &
  " </dev/null >/dev/null 2>/dev/null' 2>/dev/null || true

sleep 3
CAM_OK=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  ${NANO_USER}@${NANO_IP} \
  'ps aux | grep -v grep | grep -c arm_camera_node' 2>/dev/null || echo 0)

[ "${CAM_OK:-0}" -gt 0 ] && echo "  Câmera OK" \
  || echo "  AVISO: câmera não arrancou"

# ── [4/5] Kill nós de visão anteriores ───────────────────────────────
echo ""
echo "[4/5] Limpando nós anteriores..."
docker exec mycobot_ros2 pkill -9 -f face_detector           2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f visual_servo_controller 2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f vision_node             2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f face_follower           2>/dev/null || true
docker exec mycobot_ros2 pkill -9 -f showimage               2>/dev/null || true
sleep 1

# ── [5/5] Pipeline + câmera debug ────────────────────────────────────
echo ""
echo "[5/5] Iniciando pipeline..."

docker exec -d mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_vision_teleop 03_visual_servo.launch.py \
    use_arm_camera:=true enable_servo:=true \
    2>&1 | tee /tmp/visual_servo.log
"

sleep 6

docker exec mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 topic pub --once /visual_servo/enabled std_msgs/Bool 'data: true'
" 2>/dev/null && echo "  Servo ATIVADO" || echo "  (ative manualmente)"

if [ -n "$DISPLAY" ]; then
  docker exec -d mycobot_ros2 bash -c "
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=$CYCLONE_XML
    export DISPLAY=$DISPLAY
    source /opt/ros/galactic/setup.bash
    source /root/custom_ws/install/setup.bash
    ros2 run image_tools showimage \
      --ros-args -r image:=/face_detection/image_debug \
      2>&1 | tee /tmp/showimage.log
  " 2>/dev/null && echo "  Janela de câmera aberta"
fi

echo ""
echo "========================================"
echo "  PRONTO  |  Ctrl+C para parar tudo"
echo "========================================"
echo ""
echo "  Ajuste de ganhos em tempo real:"
echo "    docker exec mycobot_ros2 bash -c \\"
echo "      'source /opt/ros/galactic/setup.bash && source /root/custom_ws/install/setup.bash &&"
echo "       ros2 param set /visual_servo_controller kx 1.5'"
echo ""
echo "════════════════════════════════════════"
echo "  LOG (Ctrl+C para parar e limpar tudo)"
echo "════════════════════════════════════════"
sleep 1
docker exec mycobot_ros2 tail -f /tmp/visual_servo.log
