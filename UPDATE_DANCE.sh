#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " Atualizando a dança no MyCobot..."
echo "=========================================="

# 1. Envia o arquivo editado para o Nano
sshpass -p Elephant scp -o StrictHostKeyChecking=no "${SCRIPT_DIR}/listen_keyboard_service.py" er@192.168.0.250:/home/er/

# 2. Move para a pasta do sistema e reinicia o serviço invisível
sshpass -p Elephant ssh -o StrictHostKeyChecking=no er@192.168.0.250 "echo Elephant | sudo -S mv /home/er/listen_keyboard_service.py /usr/local/bin/ && sudo chmod +x /usr/local/bin/listen_keyboard_service.py && sudo systemctl restart cobot_dance.service"

echo ""
echo "✅ Sucesso! O serviço foi reiniciado."
echo "👉 Vá até o robô e aperte a tecla ESPAÇO para ver a nova dancinha!"
