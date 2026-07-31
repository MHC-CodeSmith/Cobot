#!/bin/bash
# ============================================================
# DEPLOY_TO_NANO.sh — Sincroniza o código do bridge para o Nano
#
# Use este script quando editar mycobot_bridge.py ou outros
# arquivos do mycobot_hw_interface no PC. Copia para o Nano,
# rebuilda lá, e reinicia o bridge.
# ============================================================

detect_nano_ip() {
  if [ -n "$JETSON_NANO_IP" ] && ping -c 1 -W 1 "$JETSON_NANO_IP" >/dev/null 2>&1; then
    echo "$JETSON_NANO_IP"
  elif ping -c 1 -W 1 192.168.0.62 >/dev/null 2>&1; then
    echo "192.168.0.62"
  elif ping -c 1 -W 1 192.168.0.250 >/dev/null 2>&1; then
    echo "192.168.0.250"
  else
    echo "${JETSON_NANO_IP:-192.168.0.62}"
  fi
}

NANO_USER="er"
NANO_IP=$(detect_nano_ip)
NANO_PASS="Elephant"
SRC_DIR="$(dirname "$0")/custom_ws/src/mycobot_hw_interface"
PC_IP="$(ip route get "$NANO_IP" 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i == "src") print $(i+1)}' | head -1)"
if [ -z "$PC_IP" ]; then
  PC_IP="192.168.0.79"
fi

echo "========================================"
echo "  Deploy mycobot_hw_interface → Nano (${NANO_IP})"
echo "========================================"

echo "[1/3] Copiando arquivos..."
sshpass -p "$NANO_PASS" rsync -az --delete \
  -e "ssh -o StrictHostKeyChecking=no" \
  "$SRC_DIR/" \
  "${NANO_USER}@${NANO_IP}:/home/${NANO_USER}/custom_ws/src/mycobot_hw_interface/"

echo "[2/3] Rebuilding no Nano..."
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} 'cat > ~/cyclonedds.xml' <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>${NANO_IP}</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Discovery>
            <Peers>
                <Peer address="${PC_IP}"/>
                <Peer address="${NANO_IP}"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
EOF
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} 'cat > ~/fastdds_udp.xml' <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udp_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_participant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>udp_transport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} '
  source /opt/ros/galactic/setup.bash
  cd ~/custom_ws
  colcon build --symlink-install --packages-select mycobot_hw_interface 2>&1 | tail -5
'

echo "[3/3] Reiniciando bridge no Nano..."
sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} '
  cat > ~/start_bridge.sh <<'"'"'EOS'"'"'
#!/bin/bash
# Limpeza segura: APENAS mata processos do bridge, nada genérico.
echo "Elephant" | sudo -S fuser -k /dev/ttyTHS1 2>/dev/null || true
pkill -9 -x mycobot_bridge 2>/dev/null || true
sleep 1

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml
unset CYCLONEDDS_URI

source /opt/ros/galactic/setup.bash
source $HOME/custom_ws/install/setup.bash

echo "--- BRIDGE ATIVA | FastDDS ---"
exec ros2 launch mycobot_hw_interface mycobot_hw.launch.py mock:=False baud:=1000000
EOS
  chmod +x ~/start_bridge.sh
  pkill -9 -x mycobot_bridge 2>/dev/null || true
  sleep 2
  truncate -s 0 /tmp/bridge.log
  setsid bash -c "bash ~/start_bridge.sh > /tmp/bridge.log 2>&1" < /dev/null > /dev/null 2>&1 &
  disown -a
'
sleep 6

ALIVE=$(sshpass -p "$NANO_PASS" ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP} 'pgrep -xc mycobot_bridge')
if [ "$ALIVE" -gt 0 ]; then
  echo ""
  echo "  ✓ Bridge OK — código atualizado e rodando"
else
  echo ""
  echo "  ✗ Bridge FAILED — checa: sshpass -p Elephant ssh er@${NANO_IP} 'cat /tmp/bridge.log'"
  exit 1
fi
