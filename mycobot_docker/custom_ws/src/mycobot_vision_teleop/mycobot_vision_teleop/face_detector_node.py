#!/usr/bin/env python3
# MUST come before mediapipe — prevents tkinter error in headless Docker
import matplotlib
matplotlib.use('Agg')

"""
face_detector_node.py
=====================
Detecta rostos via MediaPipe FaceMesh e publica dados estruturados.

Tópicos publicados:
  /face_detection/center   (geometry_msgs/PointStamped)
      x = center_x normalizado [0,1]   (0 = esquerda)
      y = center_y normalizado [0,1]   (0 = topo)
      z = área da bounding box [0,1]   (proxy de distância)

  /face_detection/detected  (std_msgs/Bool)
      True quando rosto detectado com confiança suficiente.

  /face_detection/image_debug  (sensor_msgs/Image)
      Frame anotado — crosshair, bounding box estimada, erro, estado.

Parâmetros:
  image_topic                  — tópico da câmera
  min_detection_confidence     — limiar FaceMesh detecção
  min_tracking_confidence      — limiar FaceMesh rastreio
  publish_debug_image          — habilita /face_detection/image_debug
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Deve bater com o QoS do arm_camera_node (best-effort, depth-1)
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class FaceDetectorNode(Node):

    def __init__(self):
        super().__init__('face_detector')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('image_topic',                '/arm_camera/image_raw')
        self.declare_parameter('min_detection_confidence',   0.5)
        self.declare_parameter('min_tracking_confidence',    0.5)
        self.declare_parameter('publish_debug_image',        True)

        det_conf    = self.get_parameter('min_detection_confidence').value
        trk_conf    = self.get_parameter('min_tracking_confidence').value
        self._debug = self.get_parameter('publish_debug_image').value
        image_topic = self.get_parameter('image_topic').value

        # ── MediaPipe FaceMesh ────────────────────────────────────────
        # max_num_faces=2: quando dois rostos visíveis, seleciona o mais próximo
        # (maior área de bounding box = mais perto da câmera)
        self._face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=2,
            refine_landmarks=False,
            min_detection_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )

        # ── Publishers ────────────────────────────────────────────────
        self._pub_center   = self.create_publisher(PointStamped, '/face_detection/center',   10)
        self._pub_detected = self.create_publisher(Bool,         '/face_detection/detected',  10)
        self._bridge       = CvBridge()
        if self._debug:
            self._pub_img = self.create_publisher(Image, '/face_detection/image_debug', 5)

        # ── Subscriber ────────────────────────────────────────────────
        # SENSOR_QOS (best-effort) deve bater com o publisher do arm_camera_node
        self.create_subscription(Image, image_topic, self._image_cb, SENSOR_QOS)
        self.get_logger().info(f'FaceDetector (FaceMesh) — subscrito a "{image_topic}"')

    # ──────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e), throttle_duration_sec=2.0)
            return
        self._process(frame, msg.header)

    def _process(self, frame, header):
        h, w = frame.shape[:2]
        rgb     = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self._face_mesh.process(rgb)

        detected  = False
        center_x  = 0.5
        center_y  = 0.5
        box_area  = 0.0
        lms       = None

        if results.multi_face_landmarks:
            detected = True

            # Seleciona o rosto mais próximo (maior área de bounding box).
            # box_area é proxy de distância: maior área = mais perto da câmera.
            best_lms  = None
            best_area = -1.0
            for face_lms in results.multi_face_landmarks:
                face_lms_list = face_lms.landmark
                xs_f = [lm.x for lm in face_lms_list]
                ys_f = [lm.y for lm in face_lms_list]
                area_f = float((max(xs_f) - min(xs_f)) * (max(ys_f) - min(ys_f)))
                if area_f > best_area:
                    best_area = area_f
                    best_lms  = face_lms_list

            lms = best_lms

            # Ponta do nariz (landmark 4) como centro do rosto selecionado
            nose     = lms[4]
            center_x = float(nose.x)
            center_y = float(nose.y)

            # Área normalizada da bounding box
            xs       = [lm.x for lm in lms]
            ys       = [lm.y for lm in lms]
            box_area = float((max(xs) - min(xs)) * (max(ys) - min(ys)))

            if len(results.multi_face_landmarks) > 1:
                self.get_logger().info(
                    f'[FACE SELECT] {len(results.multi_face_landmarks)} rostos — '
                    f'selecionado o mais próximo (área={box_area:.3f})',
                    throttle_duration_sec=1.0
                )

        # Publica estado de detecção
        self._pub_detected.publish(Bool(data=detected))

        # Publica centro + área somente quando detectado
        if detected:
            pt = PointStamped()
            pt.header   = header
            pt.point.x  = center_x
            pt.point.y  = center_y
            pt.point.z  = box_area   # área como proxy de distância
            self._pub_center.publish(pt)

        # Debug image
        if self._debug:
            # Passa todos os rostos para desenhar os rejeitados a vermelho
            all_faces = results.multi_face_landmarks if results.multi_face_landmarks else []
            self._draw_debug(frame, detected, center_x, center_y, box_area, lms, w, h, all_faces)
            img_msg             = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header      = header
            self._pub_img.publish(img_msg)

    # ──────────────────────────────────────────────────────────────────

    def _draw_debug(self, frame, detected, cx_n, cy_n, area, lms, w, h, all_faces=None):
        cx, cy = w // 2, h // 2   # pixel centro

        # Rostos rejeitados (mais distantes) a vermelho tracejado
        if all_faces and lms is not None:
            for face in all_faces:
                if face.landmark is lms:
                    continue  # skip o selecionado — desenhado abaixo
                f_lms = face.landmark
                xs_f = [lm.x for lm in f_lms]
                ys_f = [lm.y for lm in f_lms]
                rx = int(np.mean(xs_f) * w)
                ry = int(np.mean(ys_f) * h)
                side_r = int(np.sqrt((max(xs_f)-min(xs_f))*(max(ys_f)-min(ys_f))) * min(w,h))
                cv2.rectangle(frame,
                              (rx - side_r//2, ry - side_r//2),
                              (rx + side_r//2, ry + side_r//2),
                              (0, 60, 200), 1)
                cv2.putText(frame, 'LONGE', (rx - side_r//2, ry - side_r//2 - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 60, 200), 1)

        if detected and lms is not None:
            # Contorno mínimo do rosto selecionado
            for i in [10, 338, 297, 332, 284, 251, 389, 356, 454,
                      323, 361, 288, 397, 365, 379, 378, 400, 377,
                      152, 148, 176, 149, 150, 136, 172, 58, 132,
                      93, 234, 127, 162, 21, 54, 103, 67, 109]:
                lm = lms[i]
                cv2.circle(frame, (int(lm.x * w), int(lm.y * h)), 1, (0, 200, 255), -1)

            nx, ny = int(cx_n * w), int(cy_n * h)

            # Bounding box estimada do rosto
            side = int(np.sqrt(area) * min(w, h))
            cv2.rectangle(frame,
                          (nx - side // 2, ny - side // 2),
                          (nx + side // 2, ny + side // 2),
                          (0, 200, 255), 1)

            # Ponto nariz + linha ao centro
            cv2.circle(frame, (nx, ny), 6, (0, 230, 0), -1)
            cv2.circle(frame, (nx, ny), 8, (255, 255, 255), 1)
            cv2.line(frame, (cx, cy), (nx, ny), (80, 80, 220), 2)

            # Erro normalizado + nº de rostos
            n_faces = len(all_faces) if all_faces else 1
            ex = cx_n - 0.5
            ey = cy_n - 0.5
            face_label = f'PERTO (de {n_faces})' if n_faces > 1 else 'DETECTADO'
            cv2.putText(frame, f'ex={ex:+.3f}  ey={ey:+.3f}  area={area:.3f}  [{face_label}]',
                        (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (180, 180, 180), 1)

        # Crosshair central
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (160, 160, 160), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (160, 160, 160), 1)

        # Label de estado
        label, col = ('DETECTADO', (0, 230, 0)) if detected else ('SEM ROSTO', (80, 80, 80))
        cv2.putText(frame, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaceDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[face_detector] {e}')
        import traceback; traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
