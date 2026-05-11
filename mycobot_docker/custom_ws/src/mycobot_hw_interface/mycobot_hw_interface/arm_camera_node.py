#!/usr/bin/env python3
"""
arm_camera_node.py — roda no Nano
===================================
Publica frames da câmera do braço em /arm_camera/image_raw.

Fixes vs versão anterior:
  - Thread de captura separada: cap.read() não bloqueia o executor ROS2.
  - QoS SENSOR_DATA (best-effort + depth-1): drops frames se o subscriber
    está lento; nunca trava nem acumula backlog.
  - Sem subscriber → frames descartados silenciosamente, CPU ≈ zero DDS.
  - drop_count: só publica se subscriber estiver presente (evita DDS flood).
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2
from sensor_msgs.msg import Image


# QoS padrão para sensores: best-effort, last-1 — nunca trava por falta de subscribers
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class ArmCameraNode(Node):
    def __init__(self):
        super().__init__('arm_camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width',     320)
        self.declare_parameter('height',    240)
        self.declare_parameter('fps',       20)

        cam_id  = self.get_parameter('camera_id').value
        width   = self.get_parameter('width').value
        height  = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value

        # Câmera — V4L2 preferencial, fallback auto
        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            raise RuntimeError(f'Câmera {cam_id} não encontrada')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, self._fps)
        # Buffer interno de câmera = 1 → sempre frame mais recente
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Publisher best-effort — nunca trava por falta de subscribers
        self.pub = self.create_publisher(Image, '/arm_camera/image_raw', SENSOR_QOS)

        # Frame mais recente compartilhado entre threads
        self._latest_frame  = None
        self._frame_lock    = threading.Lock()
        self._running       = True

        # Thread de captura — isolada do executor ROS2
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()

        # Timer de publicação — apenas lê _latest_frame e publica
        self.create_timer(1.0 / self._fps, self._publish_frame)

        self.get_logger().info(
            f'ArmCamera iniciada — /dev/video{cam_id} {width}x{height} @ {self._fps} fps\n'
            '  Tópico: /arm_camera/image_raw (best-effort, drop-on-full)'
        )

    # ── Thread de captura (corre independente do ROS2) ────────────────

    def _capture_loop(self):
        """Captura frames continuamente e armazena o mais recente."""
        import time
        period = 1.0 / self._fps
        while self._running:
            t0 = time.monotonic()
            ret, frame = self.cap.read()
            if ret:
                with self._frame_lock:
                    self._latest_frame = frame
            # Limita ao fps configurado mesmo se cap.read() retornar rápido
            elapsed = time.monotonic() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ── Timer de publicação (no executor ROS2) ────────────────────────

    def _publish_frame(self):
        # Só publica se houver subscribers (evita flood sem consumidor)
        if self.pub.get_subscription_count() == 0:
            return

        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None   # one-shot: não repete o mesmo frame

        if frame is None:
            return

        h, w, c = frame.shape
        msg                 = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'arm_camera_frame'
        msg.height          = h
        msg.width           = w
        msg.encoding        = 'bgr8'
        msg.is_bigendian    = False
        msg.step            = w * c
        msg.data            = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self._running = False
        if self._cap_thread.is_alive():
            self._cap_thread.join(timeout=2.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[arm_camera_node] Error: {e}')
        import traceback; traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
