"""
02_face_tracking.launch.py
==========================
Sobe o vision_node (câmera + MediaPipe) e o face_follower_node juntos.

PRÉ-REQUISITO: galactic_demo já rodando (MoveIt + bridge no Nano ativos).
  ./mycobot_docker/RUN_PLANNING_PC.sh

Para rodar este launch:
  ./mycobot_docker/RUN_TELEOP.sh  (que usa este arquivo via full_teleop)

Para habilitar o movimento após o launch:
  ros2 topic pub --once /face_follower/enabled std_msgs/Bool "data: true"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Argumentos opcionais
    enable_arg = DeclareLaunchArgument(
        'start_enabled', default_value='false',
        description='Habilitar face follower automaticamente ao iniciar'
    )

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
            'publish_debug_image':       True,   # /human/image_debug para ver na tela
        }],
    )

    face_follower = Node(
        package='mycobot_vision_teleop',
        executable='face_follower_node',
        name='face_follower',
        output='screen',
        parameters=[{
            'kp_x':          0.5,    # ganho horizontal [rad/normalized_unit]
            'kp_y':          0.3,    # ganho vertical   [rad/normalized_unit]
            'deadband':      0.07,   # 7% da imagem — zona morta
            'max_delta_rad': 0.25,   # máximo movimento por step [rad]
            'rate_hz':       3.0,    # frequência do controlador [Hz]
            'j2_offset':     0.4,    # inclinação base do joint2 (olha um pouco para cima)
            'invert_x':      False,  # True se o sentido de rotação estiver invertido
        }],
    )

    return LaunchDescription([
        enable_arg,
        vision,
        face_follower,
    ])
