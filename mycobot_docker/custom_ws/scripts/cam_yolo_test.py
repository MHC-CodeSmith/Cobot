#!/usr/bin/env python3
# ============================================================
# cam_yolo_test.py — teste da câmera do cobot + modelo YOLO
#
# Roda NO NOTEBOOK (fora do docker): abre a câmera USB, roda o
# best.pt em cada frame e mostra a janela anotada com as
# detecções (classe + confiança + centro em pixels).
#
# Uso normal: ./RUN_CAMERA_TEST.sh  (instala tudo sozinho)
# Direto:     python3 cam_yolo_test.py --camera 0 --conf 0.5
#
# Teclas: q = sair | s = salvar frame em /tmp/cam_yolo_*.jpg
# ============================================================
import argparse
import os
import time

import cv2
from ultralytics import YOLO

DEFAULT_MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "best.pt")


def delivery_for(label):
    l = label.lower()
    if l.startswith("tin_valid_red"):
        return "→ delivery_red"
    if l.startswith("tin_valid_blue"):
        return "→ delivery_blue"
    if l == "tin_invalid":
        return "→ IGNORAR (lata inválida)"
    return "→ classe não usada pelo pick&place"


def main():
    ap = argparse.ArgumentParser(description="Teste câmera + YOLO (best.pt)")
    ap.add_argument("--camera", type=int, default=0, help="índice da câmera local (/dev/videoN)")
    ap.add_argument("--url", default=None,
                    help="URL de stream MJPEG (ex.: câmera no Nano via RUN_NANO_CAMERA.sh)")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="caminho do .pt")
    ap.add_argument("--conf", type=float, default=0.5, help="confiança mínima")
    args = ap.parse_args()

    print(f"Carregando modelo: {args.model}")
    model = YOLO(args.model)
    print(f"Classes do modelo: {model.names}")

    source = args.url if args.url else args.camera
    print(f"Abrindo fonte de vídeo: {source}")
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        if args.url:
            raise SystemExit(
                f"ERRO: não abriu o stream {args.url}. O servidor está no ar? "
                f"(./RUN_NANO_CAMERA.sh status)")
        raise SystemExit(
            f"ERRO: não abriu /dev/video{args.camera}. A câmera está plugada "
            f"no USB do NOTEBOOK? (ls /dev/video*) — se ela está no COBOT, "
            f"use: ./RUN_NANO_CAMERA.sh start && ./RUN_CAMERA_TEST.sh --nano")

    print("Câmera aberta. q = sair | s = salvar frame")
    last_print = 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame falhou, tentando de novo...")
            time.sleep(0.1)
            continue

        res = model.predict(frame, conf=args.conf, verbose=False)[0]
        annotated = res.plot()

        # imprime detecções no terminal (máx 2x/s para não inundar)
        now = time.time()
        if res.boxes is not None and len(res.boxes) > 0 and now - last_print > 0.5:
            last_print = now
            for box in res.boxes:
                cls = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                print(f"  {cls:20s} conf={conf:.2f} centro=({cx:.0f},{cy:.0f})px  {delivery_for(cls)}")

        cv2.imshow("Cobot camera + YOLO (q sai, s salva)", annotated)
        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        if k == ord("s"):
            path = f"/tmp/cam_yolo_{int(now)}.jpg"
            cv2.imwrite(path, annotated)
            print(f"Frame salvo: {path}")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
