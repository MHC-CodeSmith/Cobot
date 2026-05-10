import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    description_pkg = "mycobot_description"
    config_pkg = "mycobot_280_jn_moveit_config"

    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        os.path.join(get_package_share_directory(description_pkg),
                     "urdf/mycobot_280_jn/mycobot_280_jn.urdf.xacro"),
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_description_semantic_content = load_file(config_pkg, "config/mycobot_280_jn.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml = load_yaml(config_pkg, "config/kinematics.yaml")
    moveit_controllers = load_yaml(config_pkg, "config/moveit_controllers.yaml")

    moveit_controller_manager = {
        "moveit_simple_controller_manager": moveit_controllers,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    return LaunchDescription([
        # Re-carimba timestamps do Nano com clock local do PC
        # (evita que MoveIt rejeite estados como "stale")
        Node(
            package="mycobot_hw_interface",
            executable="joint_state_relay",
            name="joint_state_relay",
            output="screen"
        ),

        # Static TF: map → mycobot_base_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "map", "mycobot_base_link"]
        ),

        # Robot State Publisher (lê /joint_states)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description]
        ),

        # MoveGroup (lê /joint_states)
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                moveit_controller_manager,
                {"robot_description_kinematics": kinematics_yaml},
                {"publish_robot_description_semantic": True},
                # Timeout generoso: o myCobot físico é lento (1-3s por move)
                # Sem isso o MoveIt cancela a execução em 0.5s (TIMED_OUT).
                {"trajectory_execution.allowed_execution_duration_scaling": 5.0},
                {"trajectory_execution.allowed_goal_duration_margin": 10.0},
                {"trajectory_execution.execution_duration_monitoring": True},
            ]
        ),

        # RViz com plugin MoveIt
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(
                get_package_share_directory(config_pkg), "config/moveit.rviz")],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"robot_description_kinematics": kinematics_yaml}
            ]
        ),
    ])
