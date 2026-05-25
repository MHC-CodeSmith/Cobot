#!/usr/bin/env python3
"""
show_face.py — teste visual da câmera do braço com detecção de rosto.

Subscreve /arm_camera/image_raw (Nano via DDS domain 42),
roda MediaPipe FaceMesh e mostra janela cv2 com:
  - landmarks faciais
  - janela target (deadband verde/laranja)
  - ponto do nariz
  - vetor de erro ao centro
  - valores ex/ey em tempo real

Rodar no Docker:
  docker exec -it mycobot_ros2 bash -c "
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=/root/custom_ws/cyclonedds_pc.xml
    export DISPLAY=:1
    source /opt/ros/galactic/setup.bash
    source /root/custom_ws/install/setup.bash
    python3 /root/custom_ws/src/mycobot_vision_teleop/mycobot_vision_teleop/show_face.py
  "
"""

import matplotlib
matplotlib.use('Agg')  # DEVE vir antes de qualquer import mediapipe

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

# deadband: se nariz dentro deste raio (fração do frame) → centrado
DEADBAND = 0.06

# landmarks do FaceMesh usados para desenho mínimo
FACE_OVAL = [10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361,
             288, 397, 365, 379, 378, 400, 377, 152, 148, 176, 149,
             150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103,
             67, 109]

NOSE_TIP = 4


class ShowFace(Node):
    def __init__(self):
        super().__init__('show_face')
        self.bridge = CvBridge()

        self.face_mesh = mp.solutions.face_mesh.FaceMesh(
            max_num_faces=1,
            refine_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.sub = self.create_subscription(
            Image, '/arm_camera/image_raw', self._cb, 10)

        self.frame = None
        self.get_logger().info('show_face pronto — aguardando /arm_camera/image_raw ...')

    def _cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(str(e), throttle_duration_sec=2.0)
            return

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(rgb)

        tracking = False
        nose_x, nose_y = 0.5, 0.5

        if results.multi_face_landmarks:
            tracking = True
            lms = results.multi_face_landmarks[0].landmark

            # Oval do rosto
            pts = [(int(lms[i].x * w), int(lms[i].y * h)) for i in FACE_OVAL]
            for i in range(len(pts)):
                cv2.line(frame, pts[i], pts[(i + 1) % len(pts)], (0, 180, 255), 1)

            # Olhos (landmarks aproximados)
            for idx in [33, 133, 362, 263]:  # cantos dos olhos
                px, py = int(lms[idx].x * w), int(lms[idx].y * h)
                cv2.circle(frame, (px, py), 3, (255, 200, 0), -1)

            # Nariz
            nose = lms[NOSE_TIP]
            nose_x, nose_y = nose.x, nose.y

        # Target window (deadband)
        rx, ry = int(DEADBAND * w), int(DEADBAND * h)
        inside = tracking and abs(nose_x - 0.5) <= DEADBAND and abs(nose_y - 0.5) <= DEADBAND
        box_color = (0, 220, 0) if inside else (0, 140, 255)
        cv2.rectangle(frame, (cx - rx, cy - ry), (cx + rx, cy + ry), box_color, 2)

        # Crosshair do centro
        cv2.line(frame, (cx - 12, cy), (cx + 12, cy), (160, 160, 160), 1)
        cv2.line(frame, (cx, cy - 12), (cx, cy + 12), (160, 160, 160), 1)

        if tracking:
            nx, ny = int(nose_x * w), int(nose_y * h)
            dot_color = (0, 220, 0) if inside else (0, 60, 255)
            cv2.circle(frame, (nx, ny), 7, dot_color, -1)
            cv2.circle(frame, (nx, ny), 9, (255, 255, 255), 1)
            cv2.line(frame, (cx, cy), (nx, ny), (80, 80, 255), 2)

        # Label status
        if inside:
            label, color = 'CENTRADO', (0, 220, 0)
        elif tracking:
            label, color = 'DETECTADO', (0, 200, 255)
        else:
            label, color = 'SEM ROSTO', (0, 0, 200)

        cv2.putText(frame, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

        # Erro numérico
        if tracking:
            ex = nose_x - 0.5
            ey = nose_y - 0.5
            cv2.putText(frame, f'ex={ex:+.3f}  ey={ey:+.3f}',
                        (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        self.frame = frame


def main():
    rclpy.init()
    node = ShowFace()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.033)
            if node.frame is not None:
                cv2.imshow('Camera do Braco', node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
