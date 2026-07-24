#!/bin/bash
# ============================================================
# RUN_CAMERA_TEST.sh — abre a câmera do cobot + roda o YOLO
#
# Para testar no lab SEM o resto do stack: plugue o USB da
# câmera do cobot no NOTEBOOK e rode este script. Abre uma
# janela ao vivo com as detecções do best.pt (latas red/blue/
# inválida) e imprime o que o pick&place decidiria.
#
# Roda no host (fora do docker). Na primeira vez cria um venv
# e instala ultralytics (demora alguns minutos — torch é grande).
#
# Uso:
#   ./RUN_CAMERA_TEST.sh                 # câmera USB local (0), conf 0.5
#   ./RUN_CAMERA_TEST.sh --nano          # câmera plugada no COBOT (stream
#                                        #  do Nano — antes: RUN_NANO_CAMERA.sh start)
#   ./RUN_CAMERA_TEST.sh --camera 2      # outra câmera local
#   ./RUN_CAMERA_TEST.sh --conf 0.35
# ============================================================
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="$DIR/.yolo_venv"
NANO_STREAM="http://192.168.0.250:8080/stream.mjpg"

# Se não for passada nenhuma fonte (--nano, --camera, --url), o padrão passa a ser o stream do Nano
ARGS=()
HAS_SOURCE=false

for a in "$@"; do
  if [ "$a" = "--nano" ]; then
    ARGS+=("--url" "$NANO_STREAM")
    HAS_SOURCE=true
  elif [ "$a" = "--camera" ] || [ "$a" = "--url" ]; then
    ARGS+=("$a")
    HAS_SOURCE=true
  else
    ARGS+=("$a")
  fi
done

if [ "$HAS_SOURCE" = false ]; then
  ARGS+=("--url" "$NANO_STREAM")
fi

if [ ! -x "$VENV/bin/python" ]; then
  echo "[setup] Criando venv em $VENV (primeira vez)..."
  python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python" -c "import ultralytics, cv2" 2>/dev/null; then
  echo "[setup] Instalando ultralytics + opencv (pode demorar alguns minutos)..."
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install ultralytics opencv-python
fi

# Configura rede ROS 2 para comunicacao com o robo
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec "$VENV/bin/python" "$DIR/custom_ws/scripts/cam_yolo_test.py" "${ARGS[@]}"
