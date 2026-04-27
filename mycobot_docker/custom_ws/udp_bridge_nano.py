import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json

class UDPBridgeNano(Node):
    def __init__(self):
        super().__init__('udp_bridge_nano')
        self.pc_ip = "10.42.0.192"
        self.pc_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.subscription = self.create_subscription(JointState, 'joint_states_raw', self.callback, 10)
        self.get_logger().info(f'UDP Bridge Nano Iniciada -> Enviando para {self.pc_ip}:{self.pc_port}')

    def callback(self, msg):
        data = {
            "name": msg.name,
            "position": list(msg.position)
        }
        try:
            self.sock.sendto(json.dumps(data).encode(), (self.pc_ip, self.pc_port))
        except Exception as e:
            self.get_logger().error(f"Erro no envio UDP: {e}")

def main():
    rclpy.init()
    node = UDPBridgeNano()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
