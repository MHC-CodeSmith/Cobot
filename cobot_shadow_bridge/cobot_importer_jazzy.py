#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import json
import threading

class CobotImporter(Node):
    def __init__(self):
        super().__init__('cobot_importer_jazzy')
        self.get_logger().info('Cobot UDP Importer Started. Waiting for data...')
        
        # Publisher for the shadow robot joint states
        self.publisher_ = self.create_publisher(JointState, '/mycobot_shadow/joint_states', 10)
        
        # Setup UDP socket
        self.udp_ip = "0.0.0.0"
        self.udp_port = 9090
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        
        self.running = True
        self.thread = threading.Thread(target=self.udp_listener)
        self.thread.start()

    def udp_listener(self):
        while self.running and rclpy.ok():
            try:
                self.sock.settimeout(1.0)
                data, addr = self.sock.recvfrom(4096)
                json_data = json.loads(data.decode('utf-8'))
                
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = json_data['name']
                msg.position = json_data['position']
                
                self.publisher_.publish(msg)

                # Log a cada 10 recepções
                if not hasattr(self, 'recv_count'): self.recv_count = 0
                self.recv_count += 1
                if self.recv_count % 10 == 0:
                    self.get_logger().info(f'[UDP RECEIVED] Recebido pacote {self.recv_count}. Pos J1: {json_data["position"][0]:.2f}')

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'UDP Error: {e}')

    def destroy_node(self):
        self.running = False
        self.thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CobotImporter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
