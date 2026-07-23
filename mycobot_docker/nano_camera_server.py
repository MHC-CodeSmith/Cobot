#!/usr/bin/env python3
# ============================================================
# nano_camera_server.py — roda NO NANO: transmite a câmera USB
# do cobot como MJPEG via HTTP para a rede.
#
# É iniciado/parado pelo RUN_NANO_CAMERA.sh (do PC). Depois:
#   stream ao vivo:  http://192.168.0.250:8080/stream.mjpg
#   foto única:      http://192.168.0.250:8080/snapshot.jpg
#
# Compatível com o Python 3.6 do JetPack (sem ThreadingHTTPServer).
# ============================================================
import argparse
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2


class ThreadingHTTPServerCompat(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Camera:
    def __init__(self, device, width, height, fps, quality):
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            raise SystemExit("ERRO: nao abriu /dev/video%d" % device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.period = 1.0 / max(fps, 1)
        self.quality = quality
        self.lock = threading.Lock()
        self.jpeg = None
        t = threading.Thread(target=self._loop)
        t.daemon = True
        t.start()

    def _loop(self):
        while True:
            t0 = time.time()
            ok, frame = self.cap.read()
            if ok:
                ok2, buf = cv2.imencode(
                    ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
                if ok2:
                    with self.lock:
                        self.jpeg = buf.tobytes()
            dt = self.period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    def latest(self):
        with self.lock:
            return self.jpeg


CAM = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # silencioso

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    jpg = CAM.latest()
                    if jpg is not None:
                        self.wfile.write(b"--frame\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(jpg)))
                        self.end_headers()
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:  # snapshot
            jpg = CAM.latest()
            if jpg is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)


def main():
    global CAM
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=60)
    args = ap.parse_args()

    CAM = Camera(args.device, args.width, args.height, args.fps, args.quality)
    srv = ThreadingHTTPServerCompat(("0.0.0.0", args.port), Handler)
    print("Camera server em http://0.0.0.0:%d/stream.mjpg" % args.port)
    srv.serve_forever()


if __name__ == "__main__":
    main()
