#!/bin/bash
# ============================================================
# SAVE_POSE.sh — grava a pose atual do robô com um nome
#
# Funciona com o RUN_POSE_TUNER.sh aberto (sliders) OU com o
# stack normal rodando (mock/robô real): captura /joint_states
# e salva em custom_ws/config/arm_poses.yaml.
#
# Uso:
#   ./SAVE_POSE.sh pick
#   ./SAVE_POSE.sh place
#   ./SAVE_POSE.sh                # lista as poses salvas
# ============================================================

NAME="$1"

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
  python3 - '$NAME'" <<'PYEOF'
import sys
import time

import rclpy
import yaml
from sensor_msgs.msg import JointState

POSES_FILE = '/root/custom_ws/config/arm_poses.yaml'
JOINT_NAMES = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]

name = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None

try:
    with open(POSES_FILE) as f:
        poses = yaml.safe_load(f) or {}
except FileNotFoundError:
    poses = {}

if name is None:
    if poses:
        print('Poses salvas em arm_poses.yaml:')
        for k, v in poses.items():
            print(f'  {k}: {[round(float(x), 3) for x in v]}')
    else:
        print('Nenhuma pose salva ainda. Use: ./SAVE_POSE.sh <nome>')
    sys.exit(0)

rclpy.init()
n = rclpy.create_node('save_pose')
got = {}


def cb(msg):
    if set(JOINT_NAMES).issubset(set(msg.name)):
        idx = {j: i for i, j in enumerate(msg.name)}
        got['joints'] = [float(msg.position[idx[j]]) for j in JOINT_NAMES]


n.create_subscription(JointState, '/joint_states', cb, 10)
t0 = time.time()
while 'joints' not in got and time.time() - t0 < 5.0:
    rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node()
rclpy.shutdown()

if 'joints' not in got:
    print('ERRO: nada em /joint_states — o pose tuner (ou o stack) esta rodando?')
    sys.exit(1)

poses[name] = got['joints']
import os
os.makedirs('/root/custom_ws/config', exist_ok=True)
with open(POSES_FILE, 'w') as f:
    yaml.safe_dump(poses, f, default_flow_style=None, sort_keys=True)

deg = [round(v * 57.2958, 1) for v in got['joints']]
print(f"Pose '{name}' salva: {[round(v, 3) for v in got['joints']]} rad")
print(f"                     {deg} graus")
print(f'Arquivo: custom_ws/config/arm_poses.yaml')
PYEOF
