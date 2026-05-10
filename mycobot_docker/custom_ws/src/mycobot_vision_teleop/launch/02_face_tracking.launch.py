"""
02_face_tracking.launch.py
==========================
vision_node + face_follower_node — segue o rosto como uma cobra.

Modo câmera local (padrão):
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py

Modo câmera do braço (Nano):
  ros2 launch mycobot_vision_teleop 02_face_tracking.launch.py use_arm_camera:=true
  (requer arm_camera_node rodando no Nano — ver RUN_ARM_CAMERA_NANO.sh)

Habilitar movimento:
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_arm_camera_arg = DeclareLaunchArgument(
        'use_arm_camera', default_value='false',
        description='true = subscreve /arm_camera/image_raw (camera do braco no Nano)'
    )
    start_enabled_arg = DeclareLaunchArgument(
        'start_enabled', default_value='false',
        description='Habilitar face follower automaticamente'
    )

    use_arm_camera = LaunchConfiguration('use_arm_camera')

    vision = Node(
        package='mycobot_vision_teleop',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'camera_id':                 0,
            'width':                     640,
            'height':                    480,
            'model_complexity':          1,
            'min_detection_confidence':  0.6,
            'min_tracking_confidence':   0.6,
            'publish_debug_image':       True,
            'use_image_topic':           use_arm_camera,
            'image_topic':               '/arm_camera/image_raw',
        }],
    )

    face_follower = Node(
        package='mycobot_vision_teleop',
        executable='face_follower_node',
        name='face_follower',
        output='screen',
        parameters=[{
            'kp_x':          0.5,
            'kp_y':          0.3,
            'deadband':      0.06,
            'max_delta_rad': 0.15,
            'rate_hz':       10.0,
            'traj_ms':       120,
            'j2_offset':     0.4,
            'invert_x':      False,
        }],
    )

    return LaunchDescription([
        use_arm_camera_arg,
        start_enabled_arg,
        vision,
        face_follower,
    ])
