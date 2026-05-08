import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from pymycobot.mycobot import MyCobot
try:
    from pymycobot.mycobot280 import MyCobot280
except ImportError:
    MyCobot280 = MyCobot
import math
import time

class MyCobotBridge(Node):
    def __init__(self):
        super().__init__('mycobot_bridge')
        
        # Parâmetros
        self.declare_parameter('port', '/dev/ttyTHS1') # No Nano é THS1
        self.declare_parameter('baud', 1000000)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        
        self.get_logger().info(f'Iniciando Bridge na porta {port} com baud {baud}')
        
        try:
            try:
                self.mc = MyCobot280(port, baud)
            except:
                self.mc = MyCobot(port, baud)
            time.sleep(1.0)
            self.get_logger().info('Conexão com MyCobot estabelecida!')
        except Exception as e:
            self.get_logger().error(f'ERRO CRÍTICO DE CONEXÃO: {e}')
            exit(1)

        # Nomes padrão do MoveIt
        self.standard_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        
        # Publisher de JointStates (Topic padrão ROS 2)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states) # 20Hz
        
        # Servidor de Ação para Movimento
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        self.get_logger().info("Bridge Mestre pronto para o MoveIt!")

    def publish_joint_states(self):
        angles = self.mc.get_angles()
        if angles and len(angles) == 6:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "mycobot_base_link"
            msg.name = self.standard_names
            # Converte de GRAUS (Hardware) para RADIANOS (ROS 2)
            msg.position = [math.radians(a) for a in angles]
            self.joint_pub.publish(msg)

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Recebida trajetória do MoveIt...')
        trajectory = goal_handle.request.trajectory
        
        for point in trajectory.points:
            # Converte de RADIANOS (ROS 2) para GRAUS (Hardware)
            angles_deg = [math.degrees(p) for p in point.positions]
            # Envia para o robô (Velocidade 80)
            self.mc.send_angles(angles_deg, 80)
            # Pequeno delay para suavidade
            time.sleep(0.02)
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main():
    rclpy.init()
    node = MyCobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
