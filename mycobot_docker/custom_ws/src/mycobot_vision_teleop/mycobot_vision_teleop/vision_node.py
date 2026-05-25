#!/usr/bin/env python3
# MUST come before any mediapipe import — prevents tkinter error in headless Docker
import matplotlib
matplotlib.use('Agg')

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisionNode(Node):
    """
    Detecta rosto via MediaPipe FaceMesh e publica posição normalizada do nariz.

    Zonas:
      inner_zone  — deadband central: rosto centrado, robô para
      outer_zone  — limite de rastreio: fora disso tracking_ok=False, robô para

    Modos de entrada:
      use_image_topic=False  — câmera local (camera_id)
      use_image_topic=True   — subscreve image_topic ROS (câmera do Nano)

    Tópicos publicados:
      /human/face_center   (Point)  — nariz normalizado [0,1], só quando rastreável
      /human/tracking_ok   (Bool)   — True = rosto dentro da outer_zone
      /human/image_debug   (Image)  — frame anotado (sem cv2.imshow, sem bloqueio)
    """

    def __init__(self):
        super().__init__('vision_node')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('camera_id',                0)
        self.declare_parameter('width',                    640)
        self.declare_parameter('height',                   480)
        self.declare_parameter('min_detection_confidence', 0.5)
        self.declare_parameter('min_tracking_confidence',  0.5)
        self.declare_parameter('publish_debug_image',      True)
        self.declare_parameter('inner_zone',               0.08)  # deadband — robô para
        self.declare_parameter('outer_zone',               0.42)  # limite — fora disso para
        self.declare_parameter('use_image_topic',          False)
        self.declare_parameter('image_topic',              '/arm_camera/image_raw')

        det_conf  = self.get_parameter('min_detection_confidence').value
        trk_conf  = self.get_parameter('min_tracking_confidence').value
        self._debug     = self.get_parameter('publish_debug_image').value
        self._use_topic = self.get_parameter('use_image_topic').value

        # ── MediaPipe FaceMesh ────────────────────────────────────────
        self._face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )

        # ── Publishers ────────────────────────────────────────────────
        self.pub_face        = self.create_publisher(Point, '/human/face_center',  10)
        self.pub_tracking_ok = self.create_publisher(Bool,  '/human/tracking_ok',  10)
        self.bridge = CvBridge()
        if self._debug:
            self.pub_image = self.create_publisher(Image, '/human/image_debug', 5)

        # ── Fonte de imagem ───────────────────────────────────────────
        self.cap = None
        if self._use_topic:
            topic = self.get_parameter('image_topic').value
            self.create_subscription(Image, topic, self._topic_cb, 10)
            self.get_logger().info(f'VisionNode (FaceMesh) — subscrito a "{topic}"')
        else:
            cam_id = self.get_parameter('camera_id').value
            w      = self.get_parameter('width').value
            h      = self.get_parameter('height').value
            self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(cam_id)
            if not self.cap.isOpened():
                raise RuntimeError(f'Câmera {cam_id} não encontrada')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.create_timer(1.0 / 30.0, self._local_cb)
            self.get_logger().info(f'VisionNode (FaceMesh) — câmera local {cam_id}')

    # ── Callbacks de entrada ──────────────────────────────────────────

    def _local_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame falhou', throttle_duration_sec=2.0)
            return
        self._process(frame)

    def _topic_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e), throttle_duration_sec=2.0)
            return
        self._process(frame)

    # ── Processamento ─────────────────────────────────────────────────

    def _process(self, frame):
        inner = self.get_parameter('inner_zone').value
        outer = self.get_parameter('outer_zone').value

        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._face_mesh.process(rgb)

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        nose_x, nose_y = 0.5, 0.5
        face_detected = False

        if results.multi_face_landmarks:
            face_detected = True
            lms  = results.multi_face_landmarks[0].landmark
            nose = lms[4]  # ponta do nariz
            nose_x, nose_y = nose.x, nose.y

        # Zonas (fração do frame, medidas do centro)
        ex = nose_x - 0.5
        ey = nose_y - 0.5

        in_inner = face_detected and abs(ex) <= inner and abs(ey) <= inner
        in_outer = face_detected and abs(ex) <= outer and abs(ey) <= outer
        tracking_ok = in_outer  # só rastreável se dentro da outer zone

        # Publica dados
        self.pub_tracking_ok.publish(Bool(data=tracking_ok))
        if face_detected:
            self.pub_face.publish(Point(x=nose_x, y=nose_y, z=0.0))

        # ── Overlay visual ────────────────────────────────────────────
        if self._debug:
            if face_detected:
                # Landmarks faciais mínimos (contorno do rosto)
                for i in [10, 338, 297, 332, 284, 251, 389, 356, 454,
                          323, 361, 288, 397, 365, 379, 378, 400, 377,
                          152, 148, 176, 149, 150, 136, 172, 58, 132,
                          93, 234, 127, 162, 21, 54, 103, 67, 109]:
                    lm = lms[i]
                    cv2.circle(frame, (int(lm.x * w), int(lm.y * h)),
                               1, (0, 200, 255), -1)

            # Outer zone box — zona de rastreio (azul acinzentado)
            orx, ory = int(outer * w), int(outer * h)
            cv2.rectangle(frame,
                          (cx - orx, cy - ory), (cx + orx, cy + ory),
                          (120, 120, 80), 1)

            # Inner zone box — deadband (verde = centrado, laranja = corrigindo)
            irx, iry = int(inner * w), int(inner * h)
            if in_inner:
                inner_color = (0, 230, 0)      # verde: centrado
            elif tracking_ok:
                inner_color = (0, 140, 255)    # laranja: corrigindo
            else:
                inner_color = (60, 60, 200)    # vermelho-escuro: fora do alcance
            cv2.rectangle(frame,
                          (cx - irx, cy - iry), (cx + irx, cy + iry),
                          inner_color, 2)

            # Crosshair central
            cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (160, 160, 160), 1)
            cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (160, 160, 160), 1)

            if face_detected:
                nx, ny = int(nose_x * w), int(nose_y * h)
                dot_color = (0, 230, 0) if in_inner else (0, 80, 255)
                cv2.circle(frame, (nx, ny), 6, dot_color, -1)
                cv2.circle(frame, (nx, ny), 8, (255, 255, 255), 1)
                if not in_inner:
                    cv2.line(frame, (cx, cy), (nx, ny), (80, 80, 220), 2)

            # Status
            if in_inner:
                label, col = 'CENTRADO',  (0, 230, 0)
            elif tracking_ok:
                label, col = 'SEGUINDO',  (0, 180, 255)
            elif face_detected:
                label, col = 'FORA',      (0, 0, 200)
            else:
                label, col = 'SEM ROSTO', (80, 80, 80)
            cv2.putText(frame, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

            if face_detected:
                cv2.putText(frame, f'ex={ex:+.3f}  ey={ey:+.3f}',
                            (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (180, 180, 180), 1)

            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.pub_image.publish(img_msg)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[vision_node] {e}')
        import traceback; traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
