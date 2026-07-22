import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Pose tuner: sliders (joint_state_publisher_gui) + RViz, SEM
    bridge/move_group. Use para posar o robô manualmente e gravar as
    poses com SAVE_POSE.sh <nome>."""
    description_pkg = "mycobot_description"
    config_pkg = "mycobot_280_jn_moveit_config"

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(get_package_share_directory(description_pkg),
                     "urdf/mycobot_280_jn/mycobot_280_jn.urdf.xacro"),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "mycobot_base_link"]
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description]
        ),
        # Janela com os SLIDERS das juntas (publica /joint_states)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            parameters=[robot_description]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(
                get_package_share_directory(config_pkg), "config/pose_tuner.rviz")],
            parameters=[robot_description]
        ),
    ])
