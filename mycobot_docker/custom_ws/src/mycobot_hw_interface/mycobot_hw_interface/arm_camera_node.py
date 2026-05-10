#!/usr/bin/env python3
"""
arm_camera_node.py — roda no Nano
===================================
Publica os frames da câmera do braço como sensor_msgs/Image no tópico
/arm_camera/image_raw, usando CycloneDDS domain 42.

O vision_node no PC Docker subscreve esse tópico (use_image_topic: true)
e processa com MediaPipe sem precisar de câmera local.

Lançar no Nano:
  source /opt/ros/galactic/setup.bash
  source ~/custom_ws/install/setup.bash
  export ROS_DOMAIN_ID=42
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=~/cyclonedds.xml
  ros2 run mycobot_hw_interface arm_camera_node
"""

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image


class ArmCameraNode(Node):
    def __init__(self):
        super().__init__('arm_camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width',     640)
        self.declare_parameter('height',    480)
        self.declare_parameter('fps',       30)

        cam_id  = self.get_parameter('camera_id').value
        width   = self.get_parameter('width').value
        height  = self.get_parameter('height').value
        fps     = self.get_parameter('fps').value

        self.cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(cam_id)
        if not self.cap.isOpened():
            raise RuntimeError(f'Câmera {cam_id} não encontrada')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.pub = self.create_publisher(Image, '/arm_camera/image_raw', 5)
        self.create_timer(1.0 / fps, self._capture)

        self.get_logger().info(
            f'ArmCamera iniciada — /dev/video{cam_id} {width}x{height} @ {fps} fps\n'
            '  Tópico: /arm_camera/image_raw'
        )

    def _capture(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame falhou', throttle_duration_sec=3.0)
            return

        # Publica como raw BGR8 sem cv_bridge (evita dependência extra no Nano)
        h, w, c = frame.shape
        msg = Image()
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
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
