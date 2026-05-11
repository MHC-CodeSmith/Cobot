from setuptools import setup
import os
from glob import glob

package_name = 'mycobot_vision_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhc',
    maintainer_email='mhc@example.com',
    description='Vision-based teleoperation for MyCobot 280 JN using MediaPipe',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = mycobot_vision_teleop.vision_node:main',
            'face_follower_node = mycobot_vision_teleop.face_follower_node:main',
            'arm_mapper_node = mycobot_vision_teleop.arm_mapper_node:main',
            'target_follower_node = mycobot_vision_teleop.target_follower_node:main',
            # Visual servoing (nova arquitetura)
            'face_detector_node = mycobot_vision_teleop.face_detector_node:main',
            'visual_servo_controller_node = mycobot_vision_teleop.visual_servo_controller_node:main',
        ],
    },
)
