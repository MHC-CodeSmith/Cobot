#!/bin/bash
# ============================================================
# RUN_TEACH.sh — modo ensino no ROBÔ REAL: posicione na mão
#
# Fluxo para gravar as poses reais (pick/place/scan):
#   1. Stack real rodando (RUN_PLANNING_PC.sh, bridge no Nano)
#   2. ./RUN_TEACH.sh release   ← SEGURE O BRAÇO! ele cai solto
#   3. Posicione o copo de sucção na lata, na mão
#   4. ./SAVE_POSE.sh pick      (grava os ângulos REAIS do braço)
#   5. Repita p/ pick_approach, place, place_approach, scan...
#   6. ./RUN_TEACH.sh lock      ← trava os servos de volta
#
# ATENÇÃO: no 'release' o braço perde a força e CAI se não for
# segurado. Segure antes de rodar.
# ============================================================

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

case "${1:-}" in
  release)
    echo "SEGURE O BRAÇO — soltando servos em 3s..."
    sleep 3
    docker exec mycobot_ros2 bash -c "$ROS_ENV
      ros2 service call /release_servos std_srvs/srv/Trigger {}
    "
    ;;
  lock)
    echo "Travando servos na posição atual..."
    docker exec mycobot_ros2 bash -c "$ROS_ENV
      ros2 service call /lock_servos std_srvs/srv/Trigger {}
    "
    ;;
  interactive|"")
    echo "Iniciando modo ensino interativo..."
    docker exec -it mycobot_ros2 bash -c "$ROS_ENV
      python3 /root/custom_ws/scripts/teach_poses.py
    "
    ;;
  *)
    echo "Uso: $0 {interactive|release|lock}"
    echo "  (sem argumentos) — Inicia o menu interativo de gravação de poses"
    echo "  release          — Solta os servos imediatamente (SEGURE O BRAÇO!)"
    echo "  lock             — Trava os servos de volta"
    exit 1
    ;;
esac
