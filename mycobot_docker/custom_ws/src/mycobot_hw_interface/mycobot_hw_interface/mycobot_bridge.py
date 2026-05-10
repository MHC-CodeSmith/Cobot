import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280
import math
import time


class MyCobotBridge(Node):
    def __init__(self):
        super().__init__('mycobot_bridge')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('mock', False)
        # Velocidade para comandos diretos (face tracking).
        # 20-40 = suave para visual servoing; 70-100 = rápido mas brusco.
        self.declare_parameter('tracking_speed', 30)
        # Velocidade para trajectórias MoveIt (plan+execute).
        self.declare_parameter('moveit_speed', 60)

        self.port           = self.get_parameter('port').get_parameter_value().string_value
        self.baud           = self.get_parameter('baud').get_parameter_value().integer_value
        self.mock           = self.get_parameter('mock').get_parameter_value().bool_value
        self.tracking_speed = self.get_parameter('tracking_speed').value
        self.moveit_speed   = self.get_parameter('moveit_speed').value

        if not self.mock:
            self.mc = MyCobot280(self.port, str(self.baud))
            time.sleep(1.0)
            # Fresh mode: sempre executa o comando mais recente,
            # descartando comandos antigos na fila — essencial para
            # visual servoing contínuo (face tracking).
            try:
                self.mc.set_fresh_mode(1)
                time.sleep(0.1)
                self.get_logger().info('Fresh mode ATIVO (comandos mais recentes têm prioridade)')
            except Exception as e:
                self.get_logger().warn(f'set_fresh_mode não disponível: {e}')

        # Nomes oficiais do driver elephantrobotics
        self.joint_names = [
            "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
            "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"
        ]

        # Publica raw para o relay no PC re-carimbar com clock local
        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Controle direto — face tracking (fire-and-forget, fresh mode)
        self.cmd_sub = self.create_subscription(
            JointState, 'joint_states_commands', self.command_callback, 10)

        # Action server — MoveIt plan+execute
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            self.execute_callback)

        self.get_logger().info(
            f'MyCobot Bridge | mock={self.mock} | port={self.port} | '
            f'tracking_speed={self.tracking_speed} | moveit_speed={self.moveit_speed}')

    def command_callback(self, msg):
        """Controle direto para face tracking — executa imediatamente."""
        if self.mock:
            return
        angles = [math.degrees(x) for x in msg.position]
        if len(angles) >= 6:
            self.mc.send_angles(angles[:6], self.tracking_speed)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        if self.mock:
            msg.position = [0.0] * 6
        else:
            angles = self.mc.get_angles()
            if isinstance(angles, list) and len(angles) == 6:
                msg.position = [math.radians(x) for x in angles]
            else:
                return
        self.joint_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        """Executa trajectória do MoveIt (plan+execute do RViz)."""
        self.get_logger().info('Trajetória MoveIt recebida')
        trajectory = goal_handle.request.trajectory
        for point in trajectory.points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()
            angles_deg = [math.degrees(p) for p in point.positions]
            if not self.mock:
                self.mc.send_angles(angles_deg, self.moveit_speed)
            time.sleep(0.05)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
