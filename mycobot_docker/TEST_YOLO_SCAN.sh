#!/bin/bash
# ============================================================
# TEST_YOLO_SCAN.sh — Teste de visão YOLO na pose SCAN com GUI
#
# Comportamento:
#   1. Move o robô automaticamente para a pose 'scan'.
#   2. Abre a janela interativa da câmera com YOLO (OpenCV GUI) no host.
#   3. Ao fechar a janela ('q'), o robô retorna automaticamente para a pose HOME.
# ============================================================

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

echo "=================================================="
echo "      TESTE VISUAL DE CLASSIFICAÇÃO YOLO (SCAN)   "
echo "=================================================="

# 1. Move para a pose 'scan'
echo "[1/3] Posicionando o robô na pose SCAN..."
docker exec mycobot_ros2 bash -c "$ROS_ENV python3 /root/custom_ws/scripts/move_arm.py scan"

# Função de limpeza ao sair
cleanup() {
    echo ""
    echo "[3/3] Retornando robô para a pose HOME..."
    docker exec mycobot_ros2 bash -c "$ROS_ENV python3 /root/custom_ws/scripts/move_arm.py home"
    echo "✓ Teste de visão concluído com sucesso!"
    exit 0
}
trap cleanup EXIT INT TERM

# 2. Executa a visão com janela GUI OpenCV no notebook (host)
echo "[2/3] Abrindo janela da câmera com YOLO..."
echo "👉 Pressione a tecla 'q' na janela para encerrar o teste e retornar o robô para a pose HOME."

"$DIR/RUN_CAMERA_TEST.sh" --nano --gui
