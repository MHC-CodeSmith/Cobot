import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from ament_index_python.packages import get_package_share_directory
import yaml

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
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
    # Basic info
    description_pkg = "mycobot_description"
    config_pkg = "mycobot_280_jn_moveit_config"
    
    # 1. URDF (Robot Description)
    urdf_path = os.path.join(get_package_share_directory(description_pkg), "urdf/mycobot_280_jn/mycobot_280_jn.urdf")
    with open(urdf_path, 'r') as f:
        robot_description_content = f.read()
    robot_description = {"robot_description": robot_description_content}

    # 2. SRDF (Robot Description Semantic)
    robot_description_semantic_content = load_file(config_pkg, "config/mycobot_280_jn.srdf")
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    # 3. Kinematics YAML
    kinematics_yaml = load_yaml(config_pkg, "config/kinematics.yaml")

    # 4. Joint Limits
    joint_limits_yaml = load_yaml(config_pkg, "config/joint_limits.yaml")

    # 4b. OMPL Planning
    ompl_planning_yaml = load_yaml(config_pkg, "config/ompl_planning.yaml")

    # 4c. MoveIt Controllers
    moveit_controllers_yaml = load_yaml(config_pkg, "config/moveit_controllers.yaml")
    moveit_controller_manager = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 2.0,
        "trajectory_execution.allowed_goal_duration_margin": 1.0,
        "trajectory_execution.allowed_start_tolerance": 0.1, # Aumentado para 5 graus por causa do drift dos servos
    }

    # 5. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_yaml,
            moveit_controller_manager,
            trajectory_execution,
            {"publish_robot_description_semantic": True},
            {"planning_pipelines": ["ompl"]},
            {"default_planning_pipeline": "ompl"},
            # Parâmetros críticos para suportar o clock drift do Nano (Não mexe aqui!)
            {
                "robot_description_planning": {
                    "current_state_monitor": {
                        "timeout": 10.0,
                        "wait_for_initial_state_timeout": 30.0
                    }
                },
                "move_group": {
                    "current_state_monitor": {
                        "timeout": 10.0
                    }
                },
                "trajectory_execution": {
                    "wait_for_robot_state_timeout": 10.0,
                    "allowed_start_tolerance": 0.1
                }
            }
        ],
    )

    # 6. RViz Node
    rviz_config_file = os.path.join(get_package_share_directory(config_pkg), "config/moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
        remappings=[
            ("/tf", "/mycobot/tf"),
            ("/tf_static", "/mycobot/tf_static"),
            ("/joint_states", "/mycobot/joint_states"),
        ],
    )

    # 7. Static TF for virtual joint
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # 8. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 9. Joint State Relay (PC-side timestamp fixer)
    joint_state_relay = Node(
        package="mycobot_hw_interface",
        executable="joint_state_relay",
        output="screen",
    )

    # Wrapper for namespace
    mycobot_namespace = PushRosNamespace('mycobot')
    
    return LaunchDescription([
        GroupAction([
            mycobot_namespace,
            static_tf,
            robot_state_publisher,
            joint_state_relay,
            # mycobot_bridge,  <-- REMOVIDO PARA USAR A BRIDGE REAL DO NANO
            move_group_node,
        ]),
        rviz_node,
    ])
