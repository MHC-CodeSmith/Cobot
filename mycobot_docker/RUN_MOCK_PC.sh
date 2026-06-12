#!/bin/bash
# ============================================================
# RUN_MOCK_PC.sh — MoveIt + RViz 100% local (SEM robô/Nano)
#
# Roda o mycobot_bridge em mock DENTRO do container: ele simula
# os ângulos em memória e expõe os mesmos tópicos/serviços do
# robô real (joint_states_raw, FollowJointTrajectory, /pump_on,
# /pump_off, /pump_state). Plan & Execute no RViz funciona, e a
# pump pode ser testada com ./RUN_PUMP.sh on/off/status.
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

echo "========================================"
echo "  [1/3] Garantindo container ativo"
echo "========================================"
docker start mycobot_ros2 >/dev/null 2>&1 || true
sleep 1

echo "========================================"
echo "  [2/3] Limpando processos ros2 antigos"
echo "========================================"
docker exec mycobot_ros2 bash -c "
  ps aux | grep -E 'ros2|rviz|move_group|joint_state|robot_state|static_transform|mycobot_bridge' | grep -v grep | awk '{print \$2}' | xargs -r kill -9 2>/dev/null || true
  sleep 1
" 2>/dev/null
echo "  Docker limpo"

echo "========================================"
echo "  [3/3] Bridge MOCK + MoveIt 2 + RViz"
echo "========================================"
# Bridge mock em background (substitui o bridge do Nano)
docker exec -d mycobot_ros2 bash -c "$ROS_ENV
  ros2 run mycobot_hw_interface mycobot_bridge --ros-args -p mock:=true > /tmp/bridge_mock.log 2>&1
"
sleep 2

xhost +local:root 2>/dev/null

docker exec -it mycobot_ros2 bash -c "$ROS_ENV
  ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py
"
