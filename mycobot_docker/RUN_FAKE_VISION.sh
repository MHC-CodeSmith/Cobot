#!/bin/bash
# ============================================================
# RUN_FAKE_VISION.sh — simula o YOLO e a base móvel para testar
# o pick & place da branch featnew-yolo em MOCK, sem câmera.
#
# Publica na ordem que a rotina espera:
#   1) /product_class        (tin_valid_red | tin_valid_blue)
#   2) /pick_detection_pose  (posição da "lata" no frame da base)
#   3) espera o braço pegar, depois /delivery_state
#      (delivery_red | delivery_blue) = "base chegou"
#
# Uso (com RUN_MOCK_PC.sh rodando e o pick_and_place esperando):
#   ./RUN_FAKE_VISION.sh            # lata vermelha em (0.15, 0.12, 0.0)
#   ./RUN_FAKE_VISION.sh blue       # lata azul
#   ./RUN_FAKE_VISION.sh red 0.16 0.10 0.0
#   ./RUN_FAKE_VISION.sh invalid    # lata inválida (rotina deve IGNORAR)
# ============================================================

COLOR="${1:-red}"
X="${2:-0.15}"; Y="${3:-0.12}"; Z="${4:-0.0}"

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

if [ "$COLOR" = "invalid" ]; then
  LABEL="tin_invalid"
else
  LABEL="tin_valid_$COLOR"
fi

echo "[1/3] Publicando classe: $LABEL"
docker exec mycobot_ros2 bash -c "$ROS_ENV
  timeout 6 ros2 topic pub -r 2 /product_class std_msgs/msg/String \"{data: $LABEL}\" >/dev/null
"

echo "[2/3] Publicando posição da lata: ($X, $Y, $Z)"
docker exec mycobot_ros2 bash -c "$ROS_ENV
  timeout 6 ros2 topic pub -r 2 /pick_detection_pose geometry_msgs/msg/PointStamped \
    \"{header: {frame_id: mycobot_base_link}, point: {x: $X, y: $Y, z: $Z}}\" >/dev/null
"

if [ "$COLOR" = "invalid" ]; then
  echo "Lata inválida publicada — a rotina deve ignorar o pick. Fim."
  exit 0
fi

echo "[3/3] Aguardando o braço pegar a lata (25s)..."
sleep 25
echo "      Base 'chegou' em delivery_$COLOR"
docker exec mycobot_ros2 bash -c "$ROS_ENV
  timeout 15 ros2 topic pub -r 1 /delivery_state std_msgs/msg/String \"{data: delivery_$COLOR}\" >/dev/null
"
echo "Simulação completa."
