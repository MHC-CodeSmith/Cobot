from setuptools import find_packages, setup

package_name = 'mycobot_hw_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mycobot_hw.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhc',
    maintainer_email='carvalho.matheush@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mycobot_bridge = mycobot_hw_interface.mycobot_bridge:main',
            'joint_state_relay = mycobot_hw_interface.joint_state_relay:main',
            'udp_bridge_pc = mycobot_hw_interface.udp_bridge_pc:main'
        ],
    },
)
