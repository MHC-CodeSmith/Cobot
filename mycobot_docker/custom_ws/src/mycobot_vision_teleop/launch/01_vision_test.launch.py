import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_vision_teleop',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[{
                'camera_id': 0,
                'width': 640,
                'height': 480,
                'model_complexity': 1,
                'min_detection_confidence': 0.6,
                'min_tracking_confidence': 0.6,
                'publish_debug_image': True
            }]
        )
    ])
