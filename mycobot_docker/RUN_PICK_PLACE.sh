#!/bin/bash
# ============================================================
# RUN_PICK_PLACE.sh — rotina pick & place com a Suction Pump
#
# Pré-requisito: stack rodando (RUN_MOCK_PC.sh ou
# RUN_PLANNING_PC.sh — move_group + bridge no ar).
#
# Uso:
#   ./RUN_PICK_PLACE.sh                          # poses padrão
#   ./RUN_PICK_PLACE.sh --pick 0.15 0.12 0.0 --place 0.15 -0.12 0.0
#   ./RUN_PICK_PLACE.sh --hover 0.08 --settle 1.5
#
# Poses = pump_tcp (face do copo) no frame mycobot_base_link,
# copo apontando para baixo.
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

docker exec -it mycobot_ros2 bash -c "$ROS_ENV
  python3 /root/custom_ws/scripts/pick_and_place.py $*
"
