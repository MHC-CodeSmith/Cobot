#!/bin/bash
# ============================================================
# RUN_ROBOT_NANO.sh — Inicia Discovery Server + Bridge no Nano
# Delega ao start_bridge.sh que já existe e está configurado.
# O Nano é o servidor de descoberta (porta 11811).
# ============================================================

NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"

echo "========================================"
echo "  Ligando DS + Bridge no Nano"
echo "  SSH: ${NANO_USER}@${NANO_IP}"
echo "========================================"

sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no -t ${NANO_USER}@${NANO_IP} "~/start_bridge.sh"
