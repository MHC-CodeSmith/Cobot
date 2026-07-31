#!/bin/bash
# ============================================================
# RUN_PLANNING_PC.sh — Inicia MoveIt/RViz no Docker + bridge no Nano
#
# Garante estado DDS limpo: reinicia bridge no Nano via setsid
# (imune a SIGHUP do SSH), limpa processos antigos no Docker,
# depois lança MoveIt.
# ============================================================

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
echo "  [1/3] Reiniciando bridge no Nano (${NANO_IP})"
echo "========================================"
# Passo 1: mata processos antigos (SSH rápido, sem background)
echo "  Matando bridge anterior..."
timeout 10 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ${NANO_USER}@${NANO_IP} \
  'pkill -9 -x mycobot_bridge 2>/dev/null; pkill -9 -x arm_camera_node 2>/dev/null; truncate -s 0 /tmp/bridge.log; echo killed' \
  2>/dev/null || true

sleep 2

# Passo 2: inicia bridge — redireciona TODOS os fds do bash filho para /dev/null
# antes de disparar o nohup, para que o SSH não fique esperando os fds do processo ROS
echo "  Iniciando bridge em ${NANO_IP}..."
timeout 12 sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 ${NANO_USER}@${NANO_IP} \
  'bash -c "nohup bash ~/start_bridge.sh >/tmp/bridge.log 2>&1 </dev/null &" </dev/null >/dev/null 2>/dev/null' \
  2>/dev/null || true

sleep 6

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
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"
