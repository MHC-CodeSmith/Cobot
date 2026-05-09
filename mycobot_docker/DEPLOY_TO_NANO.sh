#!/bin/bash
# ============================================================
# DEPLOY_TO_NANO.sh — Sincroniza o código do bridge para o Nano
#
# Use este script quando editar mycobot_bridge.py ou outros
# arquivos do mycobot_hw_interface no PC. Copia para o Nano,
# rebuilda lá, e reinicia o bridge.
# ============================================================

NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"
SRC_DIR="$(dirname "$0")/custom_ws/src/mycobot_hw_interface"

echo "========================================"
echo "  Deploy mycobot_hw_interface → Nano"
echo "========================================"

echo "[1/3] Copiando arquivos..."
sshpass -p "$NANO_PASS" rsync -az --delete \
  -e "ssh -o StrictHostKeyChecking=no" \
  "$SRC_DIR/" \
  "${NANO_USER}@${NANO_IP}:/home/${NANO_USER}/custom_ws/src/mycobot_hw_interface/"

echo "[2/3] Rebuilding no Nano..."
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} '
  source /opt/ros/galactic/setup.bash
  cd ~/custom_ws
  colcon build --symlink-install --packages-select mycobot_hw_interface 2>&1 | tail -5
'

echo "[3/3] Reiniciando bridge no Nano..."
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} '
  pkill -9 -f mycobot_bridge 2>/dev/null || true
  pkill -9 -f "ros2 launch mycobot_hw_interface" 2>/dev/null || true
  sleep 2
  truncate -s 0 /tmp/bridge.log
  setsid bash -c "bash ~/start_bridge.sh > /tmp/bridge.log 2>&1" < /dev/null > /dev/null 2>&1 &
  disown -a
'
sleep 6

ALIVE=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} 'ps aux | grep -v grep | grep -c mycobot_bridge')
if [ "$ALIVE" -gt 0 ]; then
  echo ""
  echo "  ✓ Bridge OK — código atualizado e rodando"
else
  echo ""
  echo "  ✗ Bridge FAILED — checa: sshpass -p Elephant ssh er@192.168.0.250 'cat /tmp/bridge.log'"
  exit 1
fi
