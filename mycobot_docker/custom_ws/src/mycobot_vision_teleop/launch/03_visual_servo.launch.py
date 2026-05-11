"""
03_visual_servo.launch.py
==========================
Pipeline completo de visual servoing eye-in-hand para o myCobot 280 JN.

Componentes:
  1. joint_state_relay      — re-stampa joint_states do Nano com clock do PC
  2. face_detector_node     — MediaPipe FaceMesh → /face_detection/*
  3. visual_servo_controller_node — state machine IBVS → /joint_states_commands
  4. static_transform_publisher   — link6 → camera_optical_frame (calib. necessária)

Estágios de teste recomendados:
  # Stage 1 — somente detector (sem movimento do robô)
  ros2 launch mycobot_vision_teleop 03_visual_servo.launch.py enable_servo:=false
  ros2 run image_tools showimage --ros-args -r image:=/face_detection/image_debug

  # Stage 2 — ativar servo manualmente com ganhos baixos
  ros2 topic pub --once /visual_servo/enabled std_msgs/Bool "data: true"

  # Stage 3 — auto-ativado
  ros2 launch mycobot_vision_teleop 03_visual_servo.launch.py start_enabled:=true

  # Verificar estado da state machine
  ros2 topic echo /visual_servo/active
  ros2 topic echo /face_detection/center

Parâmetros de launch:
  use_arm_camera   — true (câmera do Nano) / false (webcam local)
  start_enabled    — true: ativa servo ao subir
  enable_servo     — false: sobe apenas o detector (sem enviar comandos)
  config_file      — path para YAML com parâmetros personalizados
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('mycobot_vision_teleop')
    default_config = os.path.join(pkg_share, 'config', 'visual_servo.yaml')

    # ── Argumentos de launch ──────────────────────────────────────────
    use_arm_camera_arg = DeclareLaunchArgument(
        'use_arm_camera', default_value='true',
        description='true = câmera do braço (Nano); false = webcam local'
    )
    start_enabled_arg = DeclareLaunchArgument(
        'start_enabled', default_value='false',
        description='Ativar servo automaticamente ao subir'
    )
    enable_servo_arg = DeclareLaunchArgument(
        'enable_servo', default_value='true',
        description='false = sobe apenas detector (Stage 1, sem movimento)'
    )
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='YAML com parâmetros de face_detector e visual_servo_controller'
    )

    config_file     = LaunchConfiguration('config_file')
    use_arm_camera  = LaunchConfiguration('use_arm_camera')

    # ── [1] Relay de joint_states ─────────────────────────────────────
    # Re-carimba timestamps vindos do Nano com o clock do PC.
    # visual_servo_controller precisa de /joint_states para inicializar
    # o integrador de posição ao ser ativado.
    joint_state_relay = Node(
        package='mycobot_hw_interface',
        executable='joint_state_relay',
        name='joint_state_relay',
        output='screen',
    )

    # ── [2] Face detector ─────────────────────────────────────────────
    # Publica:
    #   /face_detection/center   (PointStamped: x=cx, y=cy, z=area)
    #   /face_detection/detected (Bool)
    #   /face_detection/image_debug (Image, se publish_debug_image=true)
    face_detector = Node(
        package='mycobot_vision_teleop',
        executable='face_detector_node',
        name='face_detector',
        output='screen',
        parameters=[
            config_file,
            {'image_topic': '/arm_camera/image_raw'},  # sobrescrevível via config_file
        ],
    )

    # ── [3] Visual servo controller ───────────────────────────────────
    # State machine: WAITING_FACE → TRACKING ↔ TEMPORARY_LOST
    # Publica /joint_states_commands (JointState) → bridge no Nano
    visual_servo = Node(
        package='mycobot_vision_teleop',
        executable='visual_servo_controller_node',
        name='visual_servo_controller',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('enable_servo')),
    )

    # ── [4] Transform estático: link6 → camera_optical_frame ─────────
    # A câmera está fixada no punho (link6).
    # camera_optical_frame segue a convenção ROS:
    #   Z = para frente (eixo óptico)
    #   X = para a direita na imagem
    #   Y = para baixo na imagem
    #
    # O eixo azul do link6 no RViz (Z do frame link6) aponta para a frente
    # da câmera, portanto a rotação necessária é:
    #   +90° em torno de X (flip Y↑ → Y↓, converte Z_ros → Z_optical)
    # ATENÇÃO: esta transformação é NOMINAL e pode precisar de calibração.
    #          Ajuste os valores rpy conforme a montagem real da câmera.
    #          Para inverter horizontal/vertical: use sign_x / sign_y no YAML.
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_pub',
        arguments=[
            '0', '0', '0',           # translação (câmera coincide com link6)
            '0', '0', '0', '1',      # quaternion identidade (ajustar se necessário)
            'link6',                 # frame pai (tip do braço, SRDF: tip_link="link6")
            'camera_optical_frame',  # frame filho
        ],
        output='screen',
    )

    return LaunchDescription([
        use_arm_camera_arg,
        start_enabled_arg,
        enable_servo_arg,
        config_file_arg,
        joint_state_relay,
        face_detector,
        visual_servo,
        camera_tf,
    ])
