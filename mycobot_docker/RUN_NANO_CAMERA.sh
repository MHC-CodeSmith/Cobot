#!/bin/bash
# ============================================================
# RUN_NANO_CAMERA.sh — câmera do cobot transmitida do Nano
#
# A câmera fica plugada no USB da BASE (Nano). Este script sobe
# um servidor MJPEG no Nano; o PC assiste pela rede:
#
#   ./RUN_NANO_CAMERA.sh start    # envia e inicia o servidor
#   ./RUN_NANO_CAMERA.sh status   # verifica se está no ar
#   ./RUN_NANO_CAMERA.sh stop     # para o servidor
#
# Depois, no PC:
#   ./RUN_CAMERA_TEST.sh --nano       # YOLO ao vivo no stream
#   firefox http://192.168.0.250:8080/stream.mjpg   # só ver
# ============================================================

NANO_USER="er"
NANO_IP="192.168.0.250"
NANO_PASS="Elephant"
PORT=8080
DIR="$(cd "$(dirname "$0")" && pwd)"

SSH="sshpass -p $NANO_PASS ssh -o StrictHostKeyChecking=no ${NANO_USER}@${NANO_IP}"

case "${1:-}" in
  start)
    echo "[1/3] Enviando servidor para o Nano..."
    sshpass -p "$NANO_PASS" scp -o StrictHostKeyChecking=no \
      "$DIR/nano_camera_server.py" "${NANO_USER}@${NANO_IP}:~/nano_camera_server.py"

    echo "[2/3] Iniciando no Nano (device 0, ${PORT})..."
    $SSH '
      pkill -f nano_camera_server 2>/dev/null || true
      sleep 1
      setsid bash -c "python3 ~/nano_camera_server.py --device 0 --port '"$PORT"' > /tmp/camera.log 2>&1" < /dev/null > /dev/null 2>&1 &
      disown -a
    '
    sleep 3

    echo "[3/3] Verificando stream..."
    CODE=$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "http://${NANO_IP}:${PORT}/snapshot.jpg")
    if [ "$CODE" = "200" ]; then
      echo ""
      echo "  ✓ Câmera no ar: http://${NANO_IP}:${PORT}/stream.mjpg"
      echo "    Teste o YOLO:  ./RUN_CAMERA_TEST.sh --nano"
    else
      echo ""
      echo "  ✗ Stream não respondeu (HTTP $CODE)."
      echo "    Log do Nano: sshpass -p $NANO_PASS ssh ${NANO_USER}@${NANO_IP} 'cat /tmp/camera.log'"
      echo "    Câmera plugada no USB da base? ssh e veja: ls /dev/video*"
      exit 1
    fi
    ;;
  stop)
    $SSH 'pkill -f nano_camera_server 2>/dev/null || true'
    echo "Servidor de câmera parado."
    ;;
  status)
    CODE=$(curl -s -o /dev/null -w '%{http_code}' --max-time 5 "http://${NANO_IP}:${PORT}/snapshot.jpg")
    if [ "$CODE" = "200" ]; then
      echo "✓ No ar: http://${NANO_IP}:${PORT}/stream.mjpg"
    else
      echo "✗ Fora do ar (HTTP $CODE)"
    fi
    ;;
  *)
    echo "Uso: $0 {start|stop|status}"
    exit 1
    ;;
esac
