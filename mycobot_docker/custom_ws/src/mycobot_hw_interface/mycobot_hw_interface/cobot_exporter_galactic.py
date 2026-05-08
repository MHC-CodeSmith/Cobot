#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json

class CobotExporter(Node):
    def __init__(self):
        super().__init__('cobot_exporter_galactic')
        self.get_logger().info('Cobot UDP Exporter Started')
        
        # Tentamos vários tópicos possíveis para garantir que pegamos o sinal do Nano
        self.create_subscription(JointState, '/mycobot/joint_states', self.listener_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)
        self.create_subscription(JointState, 'joint_states_raw', self.listener_callback, 10)
        
        # Setup UDP socket
        self.udp_ip = "192.168.0.79"  # Target IP: Host PC
        self.udp_port = 9999
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def listener_callback(self, msg):
        try:
            data = {
                "name": list(msg.name),
                "position": list(msg.position)
            }
            json_data = json.dumps(data)
            self.sock.sendto(json_data.encode('utf-8'), (self.udp_ip, self.udp_port))
            # Log a cada 10 envios para não inundar o terminal
            if not hasattr(self, 'send_count'): self.send_count = 0
            self.send_count += 1
            if self.send_count % 10 == 0:
                self.get_logger().info(f'[UDP SENT] Enviado pacote {self.send_count} para o Host')
        except Exception as e:
            self.get_logger().error(f'Error sending UDP: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CobotExporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
