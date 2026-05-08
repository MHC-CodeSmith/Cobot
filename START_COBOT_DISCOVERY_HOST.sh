#!/usr/bin/env bash
set -e

# Se o Nano e o PC estão no mesmo roteador via Wi-Fi, esse deve ser o IP do wlan0 do PC
# Você pode mudar isso se estiver usando ethernet (eth0)
export COBOT_DISCOVERY_IP=192.168.0.79
export COBOT_DISCOVERY_PORT=11888

echo "[INFO] Starting MyCobot isolated FastDDS Discovery Server..."
echo "[INFO] Listening on ${COBOT_DISCOVERY_IP}:${COBOT_DISCOVERY_PORT}"

# Mata instâncias antigas de forma agressiva
pkill -9 -f "fastdds discovery" || true
pkill -9 -f "fast-discovery-server" || true

# Configura o ambiente do Jazzy no host
source /opt/ros/jazzy/setup.bash

# Inicia o discovery server
# Usamos 0.0.0.0 para aceitar conexões de todas as interfaces (incluindo Docker e Nano)
fast-discovery-server -i 0 -l 0.0.0.0 -p ${COBOT_DISCOVERY_PORT}
