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
    def __init__(self):
        super().__init__('vision_node')

        # Parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('model_complexity', 1)
        self.declare_parameter('min_detection_confidence', 0.6)
        self.declare_parameter('min_tracking_confidence', 0.6)
        self.declare_parameter('publish_debug_image', True)

        camera_id  = self.get_parameter('camera_id').value
        width      = self.get_parameter('width').value
        height     = self.get_parameter('height').value
        complexity = self.get_parameter('model_complexity').value
        det_conf   = self.get_parameter('min_detection_confidence').value
        trk_conf   = self.get_parameter('min_tracking_confidence').value
        # Cache flag so timer_callback doesn't do a param lookup at 30 Hz
        self._debug = self.get_parameter('publish_debug_image').value

        # MediaPipe setup
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            model_complexity=complexity,
            min_detection_confidence=det_conf,
            min_tracking_confidence=trk_conf,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Publishers
        self.pub_shoulder    = self.create_publisher(PointStamped, '/human/right_shoulder', 10)
        self.pub_elbow       = self.create_publisher(PointStamped, '/human/right_elbow',    10)
        self.pub_wrist       = self.create_publisher(PointStamped, '/human/right_wrist',    10)
        self.pub_face        = self.create_publisher(Point,        '/human/face_center',    10)
        self.pub_tracking_ok = self.create_publisher(Bool,         '/human/tracking_ok',    10)

        if self._debug:
            self.pub_image = self.create_publisher(Image, '/human/image_debug', 10)
            self.bridge = CvBridge()

        # Camera — must be initialised to None before open so destroy_node() is safe
        # Camera Setup
        # Try opening by index first, then by string path if it looks like one
        try:
            self.cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
        except:
            self.cap = cv2.VideoCapture(f"/dev/video{camera_id}", cv2.CAP_V4L2)

        if not self.cap or not self.cap.isOpened():
            self.get_logger().info(f"Retrying with string path /dev/video{camera_id}...")
            self.cap = cv2.VideoCapture(f"/dev/video{camera_id}", cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera {camera_id}')
            raise RuntimeError('Camera not found')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info(
            f'Vision Node started — camera {camera_id} ({width}x{height}) '
            f'mediapipe complexity={complexity}'
        )

    # ------------------------------------------------------------------
    # Timer callback — runs at ~30 Hz
    # ------------------------------------------------------------------
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame', throttle_duration_sec=2.0)
            return

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results   = self.pose.process(rgb_frame)

        tracking_ok = Bool()
        tracking_ok.data = False

        if results.pose_landmarks and results.pose_world_landmarks:
            tracking_ok.data = True
            wl = results.pose_world_landmarks.landmark
            # MediaPipe indices (person's right arm):
            #   12 = RIGHT_SHOULDER, 14 = RIGHT_ELBOW, 16 = RIGHT_WRIST
            self._publish_landmark(wl[12], self.pub_shoulder)
            self._publish_landmark(wl[14], self.pub_elbow)
            self._publish_landmark(wl[16], self.pub_wrist)

            # Face center — nose tip in normalised image coords
            nose = results.pose_landmarks.landmark[0]
            face_msg = Point(x=nose.x, y=nose.y, z=nose.z)
            self.pub_face.publish(face_msg)

            if self._debug:
                self.mp_draw.draw_landmarks(
                    frame, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS
                )

        self.pub_tracking_ok.publish(tracking_ok)

        if self._debug:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self.pub_image.publish(img_msg)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _publish_landmark(self, landmark, publisher):
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mediapipe_world'
        msg.point.x = landmark.x
        msg.point.y = landmark.y
        msg.point.z = landmark.z
        publisher.publish(msg)

    # ------------------------------------------------------------------
    # Cleanup — called by rclpy on Ctrl-C / SIGTERM / node.destroy_node()
    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Releasing camera...')
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
        print(f'[vision_node] Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
