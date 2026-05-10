#!/usr/bin/env python3
# IMPORTANT: matplotlib.use('Agg') MUST come before any mediapipe import
# to prevent tkinter/display error in headless Docker environment.
import matplotlib
matplotlib.use('Agg')

import rclpy
from rclpy.node import Node
import cv2
import mediapipe as mp
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VisionNode(Node):
    """
    Detecta pose humana via MediaPipe e publica landmarks normalizados.

    Modos de entrada (parâmetro use_image_topic):
      False (padrão) — abre câmera local via OpenCV (camera_id)
      True           — subscreve a um tópico de imagem ROS2 (image_topic)
                       Usado quando a câmera está no braço do robô (Nano).

    Tópicos publicados:
      /human/face_center    (Point)        — nariz normalizado [0,1]
      /human/tracking_ok    (Bool)         — pose detectada?
      /human/right_shoulder (PointStamped) — ombro direito (coords mundo)
      /human/right_elbow    (PointStamped) — cotovelo direito
      /human/right_wrist    (PointStamped) — pulso direito
      /human/image_debug    (Image)        — frame com landmarks desenhados
    """

    def __init__(self):
        super().__init__('vision_node')

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter('camera_id',                 0)
        self.declare_parameter('width',                     640)
        self.declare_parameter('height',                    480)
        self.declare_parameter('model_complexity',          1)
        self.declare_parameter('min_detection_confidence',  0.6)
        self.declare_parameter('min_tracking_confidence',   0.6)
        self.declare_parameter('publish_debug_image',       True)
        # Modo câmera do braço: subscreve tópico em vez de abrir câmera local
        self.declare_parameter('use_image_topic',   False)
        self.declare_parameter('image_topic',       '/arm_camera/image_raw')

        complexity = self.get_parameter('model_complexity').value
        det_conf   = self.get_parameter('min_detection_confidence').value
        trk_conf   = self.get_parameter('min_tracking_confidence').value
        self._debug = self.get_parameter('publish_debug_image').value
        self._use_topic = self.get_parameter('use_image_topic').value

        # ── MediaPipe ─────────────────────────────────────────────────
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            model_complexity=complexity,
            min_detection_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # ── Publishers ────────────────────────────────────────────────
        self.pub_shoulder    = self.create_publisher(PointStamped, '/human/right_shoulder', 10)
        self.pub_elbow       = self.create_publisher(PointStamped, '/human/right_elbow',    10)
        self.pub_wrist       = self.create_publisher(PointStamped, '/human/right_wrist',    10)
        self.pub_face        = self.create_publisher(Point,        '/human/face_center',    10)
        self.pub_tracking_ok = self.create_publisher(Bool,         '/human/tracking_ok',    10)

        self.bridge = CvBridge()
        if self._debug:
            self.pub_image = self.create_publisher(Image, '/human/image_debug', 10)

        # ── Fonte de imagem ───────────────────────────────────────────
        self.cap = None

        if self._use_topic:
            # Câmera no braço do Nano — subscreve tópico ROS2
            image_topic = self.get_parameter('image_topic').value
            self.create_subscription(Image, image_topic, self._image_topic_cb, 10)
            self.get_logger().info(
                f'Vision Node (tópico) — subscrito a "{image_topic}" | '
                f'mediapipe complexity={complexity}'
            )
        else:
            # Câmera local (PC ou Docker)
            camera_id = self.get_parameter('camera_id').value
            width     = self.get_parameter('width').value
            height    = self.get_parameter('height').value

            self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(f'/dev/video{camera_id}', cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise RuntimeError(f'Não conseguiu abrir câmera {camera_id}')

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

            self.create_timer(1.0 / 30.0, self._local_camera_cb)
            self.get_logger().info(
                f'Vision Node (câmera local) — device {camera_id} ({width}x{height}) | '
                f'mediapipe complexity={complexity}'
            )

    # ──────────────────────────────────────────────────────────────────
    # Callbacks de entrada
    # ──────────────────────────────────────────────────────────────────

    def _local_camera_cb(self):
        """Timer callback — lê frame da câmera local."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame falhou', throttle_duration_sec=2.0)
            return
        self._process_frame(frame)

    def _image_topic_cb(self, msg: Image):
        """Subscriber callback — recebe frame do Nano via ROS2."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}', throttle_duration_sec=2.0)
            return
        self._process_frame(frame)

    # ──────────────────────────────────────────────────────────────────
    # Processamento MediaPipe (comum a ambos os modos)
    # ──────────────────────────────────────────────────────────────────

    def _process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(rgb)

        tracking_ok = results.pose_landmarks is not None

        if tracking_ok:
            wl = results.pose_world_landmarks.landmark
            self._publish_landmark(wl[12], self.pub_shoulder)  # RIGHT_SHOULDER
            self._publish_landmark(wl[14], self.pub_elbow)     # RIGHT_ELBOW
            self._publish_landmark(wl[16], self.pub_wrist)     # RIGHT_WRIST

            nose = results.pose_landmarks.landmark[0]
            self.pub_face.publish(Point(x=nose.x, y=nose.y, z=nose.z))

            if self._debug:
                self.mp_draw.draw_landmarks(
                    frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS
                )

        self.pub_tracking_ok.publish(Bool(data=tracking_ok))

        if self._debug:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.pub_image.publish(img_msg)

    # ──────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────

    def _publish_landmark(self, lm, publisher):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mediapipe_world'
        msg.point.x, msg.point.y, msg.point.z = lm.x, lm.y, lm.z
        publisher.publish(msg)

    def destroy_node(self):
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Câmera liberada')
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
        print(f'[vision_node] Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
