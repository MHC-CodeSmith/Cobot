#!/bin/bash
# ============================================================
# RUN_FAKE_VISION.sh — simula o YOLO e a base móvel para testar
# o pick & place da branch featnew-yolo em MOCK, sem câmera.
#
# Publica na ordem que a rotina espera:
#   1) /product_class        (tin_valid_red | tin_valid_blue | tin_invalid)
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
WAIT_ARM="${5:-25}"   # segundos até a "base chegar" no ponto de entrega

ROS_ENV="
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset CYCLONEDDS_URI
  source /opt/ros/galactic/setup.bash
  source /root/custom_ws/install/setup.bash
"

docker exec -i mycobot_ros2 bash -c "$ROS_ENV
  python3 - $COLOR $X $Y $Z $WAIT_ARM" <<'PYEOF'
import sys
import time

import rclpy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

color = sys.argv[1]
x, y, z = (float(v) for v in sys.argv[2:5])
wait_arm = float(sys.argv[5])
label = 'tin_invalid' if color == 'invalid' else f'tin_valid_{color}'

rclpy.init()
n = rclpy.create_node('fake_vision')
pub_label = n.create_publisher(String, '/product_class', 10)
pub_pose = n.create_publisher(PointStamped, '/pick_detection_pose', 10)
pub_deliv = n.create_publisher(String, '/delivery_state', 10)

msg_label = String(data=label)
msg_pose = PointStamped()
msg_pose.header.frame_id = 'mycobot_base_link'
msg_pose.point.x, msg_pose.point.y, msg_pose.point.z = x, y, z

# 1) só o label por 2s (garante que a classe chega antes da pose)
print(f'[fake_vision] publicando classe: {label}', flush=True)
for _ in range(4):
    pub_label.publish(msg_label)
    time.sleep(0.5)

# 2) label + pose por 8s
print(f'[fake_vision] publicando pose da lata: ({x}, {y}, {z})', flush=True)
for _ in range(16):
    pub_label.publish(msg_label)
    pub_pose.publish(msg_pose)
    time.sleep(0.5)

if color == 'invalid':
    print('[fake_vision] lata invalida — a rotina deve IGNORAR o pick. Fim.', flush=True)
else:
    print(f'[fake_vision] aguardando {wait_arm:.0f}s o braco pegar a lata...', flush=True)
    time.sleep(wait_arm)
    msg_deliv = String(data=f'delivery_{color}')
    print(f'[fake_vision] base chegou: {msg_deliv.data}', flush=True)
    for _ in range(20):
        pub_deliv.publish(msg_deliv)
        time.sleep(1.0)
    print('[fake_vision] simulacao completa.', flush=True)

n.destroy_node()
rclpy.shutdown()
PYEOF
