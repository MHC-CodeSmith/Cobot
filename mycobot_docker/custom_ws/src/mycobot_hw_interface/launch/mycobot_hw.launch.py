from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyTHS1',
            description='Port for MyCobot'
        ),
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baudrate for MyCobot'
        ),
        DeclareLaunchArgument(
            'mock',
            default_value='False',
            description='Use mock mode (no hardware connection)'
        ),
        GroupAction([
            PushRosNamespace('mycobot'),
            Node(
                package='mycobot_hw_interface',
                executable='mycobot_bridge',
                name='mycobot_bridge',
                output='screen',
                parameters=[{
                    'port': LaunchConfiguration('port'),
                    'baud': LaunchConfiguration('baud'),
                    'mock': LaunchConfiguration('mock')
                }]
            )
        ])
    ])
