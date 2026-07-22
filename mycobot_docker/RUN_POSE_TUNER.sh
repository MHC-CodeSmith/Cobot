#!/bin/bash
# ============================================================
# RUN_POSE_TUNER.sh — posicione o robô com SLIDERS (sem robô real)
#
# Abre uma janela com um slider por junta + RViz mostrando o robô
# (com a pump). Arraste os sliders até a pose desejada e, em OUTRO
# terminal, grave a pose com nome:
#
#   ./SAVE_POSE.sh pick     # pose de pegar a lata
#   ./SAVE_POSE.sh place    # pose de soltar
#   ./SAVE_POSE.sh pick_approach   (opcional, aproximação por cima)
#   ./SAVE_POSE.sh place_approach  (opcional)
#
# As poses vão para custom_ws/config/arm_poses.yaml e são usadas por:
#   ./RUN_PICK_PLACE.sh --pick-pose pick --place-pose place
#
# Obs: NÃO rode junto com RUN_MOCK_PC.sh (os dois publicam
# /joint_states). Feche um antes de abrir o outro.
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

echo "[1/4] Garantindo container ativo"
docker start mycobot_ros2 >/dev/null 2>&1 || true
sleep 1

echo "[2/4] Parando stack anterior (mock/move_group/rviz)"
docker exec mycobot_ros2 bash -c "
  ps aux | grep -E 'ros2|rviz|move_group|joint_state|robot_state|static_transform|mycobot_bridge' | grep -v grep | awk '{print \$2}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
" 2>/dev/null

echo "[3/4] Garantindo joint_state_publisher_gui instalado"
docker exec mycobot_ros2 bash -c "
  source /opt/ros/galactic/setup.bash
  ros2 pkg prefix joint_state_publisher_gui >/dev/null 2>&1 || \
    (apt-get update -qq && apt-get install -y -qq ros-galactic-joint-state-publisher-gui)
" || echo "  AVISO: instalacao falhou — veja erro acima"

echo "[4/4] Build + abrindo sliders e RViz"
xhost +local:root 2>/dev/null

docker exec -it mycobot_ros2 bash -c "$ROS_ENV
  cd /root/custom_ws && colcon build --packages-select mycobot_280_jn_moveit_config >/dev/null 2>&1
  source /root/custom_ws/install/setup.bash
  ros2 launch mycobot_280_jn_moveit_config pose_tuner.launch.py
"
