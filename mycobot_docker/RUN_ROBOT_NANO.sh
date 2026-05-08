#!/bin/bash
# ============================================================
# RUN_ROBOT_NANO.sh — Inicia a bridge de hardware no Jetson Nano
# (O Nano agora inicia seu próprio Discovery Server)
# ============================================================

echo "========================================"
echo "  Ligando bridge de hardware no Nano (SERVER MODE)"
echo "========================================"

NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"

# Limpa o Discovery Server local do PC (se houver) para evitar conflitos
pkill -f fastdds || true

# Dispara o script no Nano
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} "./start_bridge.sh"
