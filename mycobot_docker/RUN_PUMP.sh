#!/bin/bash
# ============================================================
# RUN_PUMP.sh — Controle da Suction Pump V2.0
#
# Pré-requisito: bridge no Nano rodando (RUN_PLANNING_PC.sh
# ou RUN_VISUAL_SERVO.sh já iniciado).
#
# Uso:
#   ./RUN_PUMP.sh on   — ativa sucção
#   ./RUN_PUMP.sh off  — desativa + libera objeto
#   ./RUN_PUMP.sh status — mostra estado atual
# ============================================================

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

CMD="${1:-status}"

case "$CMD" in
  on)
    echo "Pump ON — ativando sucção..."
    docker exec mycobot_ros2 bash -c "$ROS_ENV
      ros2 service call /pump_on std_srvs/srv/Trigger {}
    "
    ;;
  off)
    echo "Pump OFF — liberando objeto..."
    docker exec mycobot_ros2 bash -c "$ROS_ENV
      ros2 service call /pump_off std_srvs/srv/Trigger {}
    "
    ;;
  status)
    echo "Pump state:"
    # galactic não tem 'ros2 topic echo --once'
    docker exec mycobot_ros2 bash -c "$ROS_ENV
      timeout 3 ros2 topic echo /pump_state | head -2
    "
    ;;
  test)
    echo "Iniciando teste de hardware da bomba de sucção..."
    docker exec -it mycobot_ros2 bash -c "$ROS_ENV
      python3 /root/custom_ws/scripts/test_pump.py
    "
    ;;
  *)
    echo "Uso: $0 {on|off|status|test}"
    echo "  on     — liga sucção manualmente"
    echo "  off    — desliga sucção e libera objeto"
    echo "  status — mostra o estado atual"
    echo "  test   — executa ciclo completo de teste pick & place físico"
    exit 1
    ;;
esac
