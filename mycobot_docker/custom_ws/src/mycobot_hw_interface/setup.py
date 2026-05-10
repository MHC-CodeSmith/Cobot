from setuptools import setup
package_name = 'mycobot_hw_interface'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhc',
    description='MyCobot Hardware Interface',
    license='TODO',
    entry_points={
        'console_scripts': [
            'mycobot_bridge = mycobot_hw_interface.mycobot_bridge:main',
            'joint_state_relay = mycobot_hw_interface.joint_state_relay:main',
            'arm_camera_node = mycobot_hw_interface.arm_camera_node:main',
        ],
    },
)
