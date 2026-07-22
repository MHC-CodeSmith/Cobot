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
#   ./RUN_CAMERA_TEST.sh                 # câmera 0, conf 0.5
#   ./RUN_CAMERA_TEST.sh --camera 2      # outra câmera (ls /dev/video*)
#   ./RUN_CAMERA_TEST.sh --conf 0.35
# ============================================================
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
VENV="$DIR/.yolo_venv"

if [ ! -x "$VENV/bin/python" ]; then
  echo "[setup] Criando venv em $VENV (primeira vez)..."
  python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python" -c "import ultralytics, cv2" 2>/dev/null; then
  echo "[setup] Instalando ultralytics + opencv (pode demorar alguns minutos)..."
  "$VENV/bin/pip" install --quiet --upgrade pip
  "$VENV/bin/pip" install ultralytics opencv-python
fi

exec "$VENV/bin/python" "$DIR/custom_ws/scripts/cam_yolo_test.py" "$@"
