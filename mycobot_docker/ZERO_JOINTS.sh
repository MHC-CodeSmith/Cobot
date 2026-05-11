#!/bin/bash
# ZERO_JOINTS.sh — Leva todas as juntas a 0° a velocidade baixa.
# Desativa o visual servo antes, para não haver conflito.
# Uso: ./ZERO_JOINTS.sh [velocidade 1-100, default 25]

CYCLONE_XML="/root/custom_ws/cyclonedds_pc.xml"
SPEED=${1:-25}

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=$CYCLONE_XML
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

echo "========================================"
echo "  ZERO JOINTS  |  velocidade=$SPEED"
echo "========================================"

# 1. Desativa visual servo (se estiver ativo)
echo "[1/3] Desativando visual servo..."
docker exec mycobot_ros2 bash -c "$ROS_ENV
  ros2 topic pub --once /visual_servo/enabled std_msgs/msg/Bool 'data: false' 2>/dev/null
" 2>/dev/null || true
sleep 1

# 2. Envia posição zero
echo "[2/3] Enviando posição zero (speed=$SPEED)..."

# O bridge usa posição por índice (ignora nomes), velocidade via tracking_speed.
# Publicamos 3x para garantir que o bridge recebe.
for i in 1 2 3; do
  docker exec mycobot_ros2 bash -c "$ROS_ENV
    ros2 topic pub --once /joint_states_commands sensor_msgs/msg/JointState \
      '{name: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3, joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6],
        position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}' 2>/dev/null
  " 2>/dev/null || true
  sleep 0.5
done

# 3. Aguarda execução
echo "[3/3] Aguardando execução (~3s)..."
sleep 3

echo ""
echo "Feito. Verifica no braço se chegou a 0°."
echo "Para visual servo: ./RUN_VISUAL_SERVO.sh"
