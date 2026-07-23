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
import threading
import cv2
from ultralytics import YOLO

# Tenta otimizar CUDA no topo do arquivo se disponível
try:
    import torch
    if torch.cuda.is_available():
        torch.backends.cudnn.benchmark = True
        print("CUDA disponível — PyTorch CUDNN benchmark ativado")
except ImportError:
    pass

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


class ThreadedVideoCapture:
    def __init__(self, source):
        self.cap = cv2.VideoCapture(source)
        self.ret = False
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    def _reader(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            with self.lock:
                self.ret = ret
                self.frame = frame

    def read(self):
        with self.lock:
            if self.frame is None:
                return False, None
            return self.ret, self.frame.copy()

    def isOpened(self):
        return self.cap.isOpened()

    def release(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.cap.release()


class AsyncYOLOInference:
    def __init__(self, model_path, conf_threshold):
        self.model = YOLO(model_path)
        self.conf = conf_threshold
        self.frame = None
        self.results = None
        self.lock = threading.Lock()
        self.running = True
        self.new_frame_event = threading.Event()
        
        self.thread = threading.Thread(target=self._inference_loop)
        self.thread.daemon = True
        self.thread.start()

    def update_frame(self, frame):
        with self.lock:
            self.frame = frame.copy()
        self.new_frame_event.set()

    def _inference_loop(self):
        while self.running:
            if not self.new_frame_event.wait(timeout=0.1):
                continue
            
            self.new_frame_event.clear()
            
            with self.lock:
                if self.frame is None:
                    continue
                frame_to_process = self.frame.copy()
            
            res = self.model.predict(frame_to_process, conf=self.conf, verbose=False, imgsz=640)[0]
            
            with self.lock:
                self.results = res

    def get_latest_results(self):
        with self.lock:
            return self.results

    def stop(self):
        self.running = False
        self.new_frame_event.set() # acorda a thread se estiver esperando
        self.thread.join(timeout=1.0)


def draw_predictions(frame, results, model_names):
    if results is None or results.boxes is None:
        return frame
    
    annotated_frame = frame.copy()
    for box in results.boxes:
        x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
        cls_id = int(box.cls[0])
        cls_name = model_names[cls_id]
        conf = float(box.conf[0])
        
        # Cor baseada na classe (BGR)
        if "red" in cls_name.lower():
            color = (0, 0, 255)  # Vermelho
        elif "blue" in cls_name.lower():
            color = (255, 0, 0)  # Azul
        else:
            color = (0, 255, 0)  # Verde / Outros
            
        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
        label = f"{cls_name} {conf:.2f}"
        cv2.putText(annotated_frame, label, (x1, max(y1 - 10, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                    
    return annotated_frame


def main():
    ap = argparse.ArgumentParser(description="Teste câmera + YOLO (best.pt) - Async 30 FPS")
    ap.add_argument("--camera", type=int, default=0, help="índice da câmera local (/dev/videoN)")
    ap.add_argument("--url", default=None,
                    help="URL de stream MJPEG (ex.: câmera no Nano via RUN_NANO_CAMERA.sh)")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="caminho do .pt")
    ap.add_argument("--conf", type=float, default=0.5, help="confiança mínima")
    args = ap.parse_args()

    print(f"Carregando modelo YOLO: {args.model}")
    yolo_async = AsyncYOLOInference(args.model, args.conf)
    print(f"Classes do modelo: {yolo_async.model.names}")

    source = args.url if args.url else args.camera
    print(f"Abrindo fonte de vídeo com ThreadedVideoCapture: {source}")
    cap = ThreadedVideoCapture(source)
    if not cap.isOpened():
        yolo_async.stop()
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
    
    # Loop de Exibição / Rendering a ~30 FPS
    while True:
        t0 = time.time()
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.01)
            continue

        # Atualiza o frame da thread de inferência
        yolo_async.update_frame(frame)
        
        # Recupera as últimas detecções conhecidas (sem bloquear)
        res = yolo_async.get_latest_results()
        
        # Desenha as detecções sobre o frame fresco atual
        annotated = draw_predictions(frame, res, yolo_async.model.names)

        # Imprime detecções no terminal de forma controlada
        now = time.time()
        if res is not None and res.boxes is not None and len(res.boxes) > 0 and now - last_print > 0.5:
            last_print = now
            for box in res.boxes:
                cls = yolo_async.model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                print(f"  {cls:20s} conf={conf:.2f} centro=({cx:.0f},{cy:.0f})px  {delivery_for(cls)}")

        cv2.imshow("Cobot camera + YOLO Async (q sai, s salva)", annotated)
        
        # Limita taxa de loop para ~30 FPS
        elapsed = time.time() - t0
        delay = max(int((0.033 - elapsed) * 1000), 1)
        k = cv2.waitKey(delay) & 0xFF
        
        if k == ord("q"):
            break
        if k == ord("s"):
            path = f"/tmp/cam_yolo_{int(now)}.jpg"
            cv2.imwrite(path, annotated)
            print(f"Frame salvo: {path}")

    cap.release()
    yolo_async.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
