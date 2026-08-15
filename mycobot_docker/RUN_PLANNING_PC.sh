#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia MoveIt/RViz no Docker + bridge no Nano
# ============================================================

export DISPLAY="${DISPLAY:-:0}"
xhost +local:root 2>/dev/null || true
xhost +local:docker 2>/dev/null || true

detect_nano_ip() {
  if [ -n "$JETSON_NANO_IP" ] && ping -c 1 -W 1 "$JETSON_NANO_IP" >/dev/null 2>&1; then
    echo "$JETSON_NANO_IP"
  elif ping -c 1 -W 1 192.168.0.62 >/dev/null 2>&1; then
    echo "192.168.0.62"
  elif ping -c 1 -W 1 192.168.0.250 >/dev/null 2>&1; then
    echo "192.168.0.250"
  else
    echo "${JETSON_NANO_IP:-192.168.0.62}"
  fi
}

NANO_USER="er"
NANO_IP=$(detect_nano_ip)
NANO_PASS="Elephant"

echo "========================================"
echo "  [1/4] Garantindo contêiner Docker rodando"
echo "========================================"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if docker inspect mycobot_ros2 >/dev/null 2>&1; then
  # O backend monta apenas o socket e o cliente Docker; nesse ambiente o
  # plugin `docker compose` pode não existir, mas o contêiner já criado existe.
  docker start mycobot_ros2 >/dev/null
elif docker compose version >/dev/null 2>&1; then
  docker compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
elif command -v docker-compose >/dev/null 2>&1; then
  docker-compose -f "${SCRIPT_DIR}/docker-compose.yml" up -d
else
  echo "  Docker FAILED — contêiner mycobot_ros2 ausente e Compose indisponível."
  exit 1
fi

if [ "$(docker inspect -f '{{.State.Running}}' mycobot_ros2 2>/dev/null)" != "true" ]; then
  echo "  Docker FAILED — mycobot_ros2 não confirmou estado Running."
  exit 1
fi
echo "  Docker OK (mycobot_ros2 em execução)"

echo ""
echo "========================================"
echo "  [2/4] Reiniciando bridge e câmera no Nano (${NANO_IP})"
echo "========================================"
timeout 10 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ${NANO_USER}@${NANO_IP} \
  'echo Elephant | sudo -S fuser -k /dev/ttyTHS1 2>/dev/null; pkill -9 -f mycobot_bridge 2>/dev/null; truncate -s 0 /tmp/bridge.log' \
  2>/dev/null || true

sleep 2

# Inicia bridge na Jetson Nano
timeout 12 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ${NANO_USER}@${NANO_IP} \
  'bash -c "nohup bash ~/start_bridge.sh >/tmp/bridge.log 2>&1 </dev/null &" </dev/null >/dev/null 2>/dev/null' \
  2>/dev/null || true

# Garante servidor de câmera OpenCV ativo na porta 8080
timeout 12 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ${NANO_USER}@${NANO_IP} \
  'bash -c "pgrep -f nano_camera_server || nohup python3 /home/er/nano_camera_server.py --device 0 --port 8080 >/tmp/camera.log 2>&1 &"' \
  2>/dev/null || true

sleep 4

BRIDGE_OK=$(timeout 8 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=6 ${NANO_USER}@${NANO_IP} \
  'pgrep -f mycobot_bridge | wc -l' 2>/dev/null)
BRIDGE_OK=${BRIDGE_OK:-0}
if [ "${BRIDGE_OK}" -gt 0 ]; then
  echo "  Bridge OK (mycobot_bridge rodando no Nano em ${NANO_IP})"
else
  echo "  Bridge FAILED — verifique: sshpass -p Elephant ssh er@${NANO_IP} 'cat /tmp/bridge.log'"
  exit 1
fi

echo ""
echo "========================================"
echo "  [3/4] Limpando processos ROS 2 antigos no Docker"
echo "========================================"
docker exec mycobot_ros2 bash -c "
  ps aux | grep -E 'ros2|rviz|move_group|joint_state|robot_state|static_transform' | grep -v grep | awk '{print \$2}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
" 2>/dev/null
echo "  Docker limpo"

echo ""
echo "========================================"
echo "  [4/4] Lançando MoveIt 2 + RViz2"
echo "========================================"
if ! docker exec -d -e DISPLAY="${DISPLAY:-:0}" mycobot_ros2 bash -c "
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"; then
  echo "  MoveIt FAILED — docker exec não foi aceito."
  exit 1
fi

echo "  Solicitação MoveIt/RViz aceita pelo contêiner."
