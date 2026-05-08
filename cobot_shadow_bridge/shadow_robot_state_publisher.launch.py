import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable

from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import ExecuteProcess

def generate_launch_description():
    # Caminho do workspace do cobot para pegar o URDF do Galactic e usar no Humble
    urdf_path = "/home/mhc/Germany/Cobot/mycobot_docker/custom_ws/src/mycobot_description/urdf/mycobot_280_jn/mycobot_280_jn.urdf.xacro"
    
    robot_description_content = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", urdf_path
    ]), value_type=str)

    return LaunchDescription([
        # Robot State Publisher publicando o shadow robot
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='shadow_robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}],
            remappings=[
                ('/joint_states', '/mycobot_shadow/joint_states'),
            ]
        ),
        
        # O Importer UDP executado diretamente pois não está em um pacote
        ExecuteProcess(
            cmd=['python3', '/home/mhc/Germany/Cobot/cobot_shadow_bridge/cobot_importer_jazzy.py'],
            name='cobot_importer_jazzy',
            output='screen'
        ),
        
        # TF estático: map (TurtleBot) -> mycobot_base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='shadow_cobot_anchor',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'mycobot_base_link']
        )
    ])
