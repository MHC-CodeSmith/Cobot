import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json

class UDPBridgePC(Node):
    def __init__(self):
        super().__init__('udp_bridge_pc')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', 9999))
        self.sock.settimeout(1.0)
        self.get_logger().info('UDP Bridge PC Iniciada -> Ouvindo na porta 9999')
        self.timer = self.create_timer(0.01, self.receive_callback)

    def receive_callback(self):
        try:
            data, addr = self.sock.recvfrom(4096)
            payload = json.loads(data.decode())
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = payload["name"]
            msg.position = payload["position"]
            self.publisher.publish(msg)
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"Erro na recepção UDP: {e}")

def main():
    rclpy.init()
    node = UDPBridgePC()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
