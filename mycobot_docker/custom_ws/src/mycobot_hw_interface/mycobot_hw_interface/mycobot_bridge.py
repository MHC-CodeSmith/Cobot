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

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.mock = self.get_parameter('mock').get_parameter_value().bool_value

        if not self.mock:
            self.mc = MyCobot280(self.port, str(self.baud))
            time.sleep(1.0)

        # Nomes oficiais do driver elephantrobotics
        self.joint_names = [
            "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
            "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"
        ]

        # Publica raw para o relay no PC re-carimbar com clock local
        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz

        # Subscriber para controle direto (GUI / teleop)
        self.cmd_sub = self.create_subscription(
            JointState, 'joint_states_commands', self.command_callback, 10)

        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            self.execute_callback)

        self.get_logger().info(
            f'MyCobot Bridge ativa | mock={self.mock} | port={self.port} | baud={self.baud}')

    def command_callback(self, msg):
        if self.mock:
            return
        angles = [math.degrees(x) for x in msg.position]
        if len(angles) >= 6:
            self.mc.send_angles(angles[:6], 80)

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
        self.get_logger().info('Trajetória recebida do MoveIt')
        trajectory = goal_handle.request.trajectory
        for point in trajectory.points:
            angles_deg = [math.degrees(p) for p in point.positions]
            if not self.mock:
                self.mc.send_angles(angles_deg, 80)
            time.sleep(0.02)
        goal_handle.succeed()
        return FollowJointTrajectory.Result()

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
