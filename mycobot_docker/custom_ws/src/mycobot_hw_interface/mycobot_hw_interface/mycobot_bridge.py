import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from pymycobot.mycobot import MyCobot
# Tenta importar MyCobot280 como alternativa recomendada para a série 280
try:
    from pymycobot.mycobot280 import MyCobot280
except ImportError:
    MyCobot280 = MyCobot
import math
import time

class MyCobotBridge(Node):
    def __init__(self):
        super().__init__('mycobot_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyTHS1') # Jetson Nano serial port
        self.declare_parameter('baud', 115200)
        self.declare_parameter('mock', False)
        
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.mock = self.get_parameter('mock').get_parameter_value().bool_value
        
        if not self.mock:
            self.get_logger().info(f'Connecting to MyCobot on {self.port} at {self.baud}...')
            try:
                # MyCobot280 costuma ser mais estável em versões novas
                self.mc = MyCobot280(self.port, str(self.baud)) 
                time.sleep(1.0) # Tempo para estabilizar a conexão
                
                # Testa a versão para confirmar comunicação
                version = self.mc.get_system_version()
                self.get_logger().info(f'Connected! System Version: {version}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect: {e}')
                self.get_logger().warn('Switching to MOCK mode due to connection failure.')
                self.mock = True
        else:
            self.get_logger().info("Running in MOCK mode (No physical hardware connection).")
        
        # Joint names (must match URDF - revolute joints only)
        self.joint_names = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6"
        ]
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states) # 10Hz para evitar travar a Serial
        
        # Action Server - Removida barra inicial para respeitar namespace
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'mycobot_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        self.get_logger().info("MyCobot Hardware Bridge Ready!")

    def rad_to_deg(self, rad_list):
        return [math.degrees(r) for r in rad_list]

    def deg_to_rad(self, deg_list):
        return [math.radians(d) for d in deg_list]

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        
        if self.mock:
            # In mock mode, we just publish the last commanded positions (or zeros)
            if not hasattr(self, 'current_angles'):
                self.current_angles = [0.0] * 6
            msg.position = self.current_angles
            self.joint_pub.publish(msg)
            return

        try:
            angles = self.mc.get_angles()
            if isinstance(angles, list) and len(angles) == 6:
                msg.position = self.deg_to_rad(angles)
                self.joint_pub.publish(msg)
            else:
                self.get_logger().warn('Erro de comunicação (Serial Timeout). Verifique baudrate!', throttle_duration_sec=5.0)
        except Exception as e:
            self.get_logger().error(f"Failed to get angles: {e}")

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory...')
        trajectory = goal_handle.request.trajectory
        
        if not trajectory.points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        last_time = 0.0
        for point in trajectory.points:
            if not rclpy.ok():
                break
            
            # Calcula o tempo esperado do MoveIt para este ponto
            curr_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            dt = curr_time - last_time
            
            if self.mock:
                self.current_angles = point.positions
                if dt > 0:
                    time.sleep(dt)
            else:
                # Filtragem: O myCobot não processa bem mais que 20-30Hz via serial
                # Se o ponto for muito próximo do anterior (menos de 0.04s), pulamos ele
                if dt < 0.04 and point != trajectory.points[-1]:
                    continue
                
                if dt > 0:
                    time.sleep(dt)
                
                angles_deg = self.rad_to_deg(point.positions)
                # Velocidade dinâmica: quanto maior o passo, mais rápido
                # Mas para trajetórias do MoveIt, 80-100 costuma ser bom para seguir o fluxo
                self.mc.send_angles(angles_deg, 80) 
            
            last_time = curr_time
        
        # Pequeno delay para garantir que o último movimento foi processado pelo firmware
        time.sleep(0.5) 

        self.get_logger().info('Trajectory execution complete.')
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MyCobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
