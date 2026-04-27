#!/bin/bash
# NANO_IP="10.42.0.1"
# PC_IP (Ethernet/WiFi para o Discovery Server)
DISCOVERY_IP="192.168.0.79" # Mude para o IP do seu PC na rede local onde o TurtleBot está

echo "--- INICIANDO DISCOVERY SERVER ---"
# Roda o discovery server em background no PC
pkill -f "fastdds discovery" || true
fastdds discovery -i 0 -l $DISCOVERY_IP -p 11811 > /tmp/fastdds_discovery.log 2>&1 &
sleep 2

echo "--- INICIANDO LIMPEZA TOTAL ---"
# Limpa processos que o usuário mhc é dono
pkill -u $USER -f ros2 || true
pkill -u $USER -f move_group || true
pkill -u $USER -f joint_state_relay || true

# Limpa PC (Docker) - O Docker exec já roda como root lá dentro
docker exec mycobot_ros2 pkill -9 -f ros2 || true

# Limpa Nano via SSH
sshpass -p 'Elephant' ssh -o StrictHostKeyChecking=no er@192.168.0.250 "pkill -9 -u er -f ros2; pkill -9 -u er -f python3; rm -f /var/lock/LCK*; rm -rf ~/.ros/discovery_server/"

echo "--- INICIANDO ROBÔ (NANO) ---"
sshpass -p 'Elephant' ssh -o StrictHostKeyChecking=no er@192.168.0.250 "export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DISCOVERY_SERVER=$DISCOVERY_IP:11811 && source /opt/ros/galactic/setup.bash && source ~/custom_ws/install/setup.bash && nohup ros2 launch mycobot_hw_interface mycobot_hw.launch.py mock:=False baud:=1000000 > /tmp/nano_bridge.log 2>&1 &"

echo "Esperando o Robô (10s)..."
sleep 10
sshpass -p 'Elephant' ssh -o StrictHostKeyChecking=no er@192.168.0.250 "grep -i 'Ready' /tmp/nano_bridge.log"

echo "--- INICIANDO MOVEIT (PC DOCKER) ---"
docker exec -it mycobot_ros2 bash -c "export ROS_DOMAIN_ID=42 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_DISCOVERY_SERVER=$DISCOVERY_IP:11811 && source /opt/ros/galactic/setup.bash && source /root/custom_ws/install/setup.bash && ros2 launch mycobot_280_jn_moveit_config galactic_demo.launch.py"
