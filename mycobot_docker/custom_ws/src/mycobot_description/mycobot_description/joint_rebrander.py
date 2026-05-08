#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointRebrander(Node):
    def __init__(self):
        super().__init__('joint_rebrander')
        # Subscreve no tópico global onde o Nano publica
        self.sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        # Publica no tópico que o MoveIt vai ler
        self.pub = self.create_publisher(JointState, '/joint_states_clean', 10)
        
        # Mapa: Nome do Nano -> Nosso novo nome
        self.map = {
            "joint2_to_joint1": "cobot_joint_1",
            "joint3_to_joint2": "cobot_joint_2",
            "joint4_to_joint3": "cobot_joint_3",
            "joint5_to_joint4": "cobot_joint_4",
            "joint6_to_joint5": "cobot_joint_5",
            "joint6output_to_joint6": "cobot_joint_6",
            # Caso o Nano já mande joint1...
            "joint1": "cobot_joint_1",
            "joint2": "cobot_joint_2",
            "joint3": "cobot_joint_3",
            "joint4": "cobot_joint_4",
            "joint5": "cobot_joint_5",
            "joint6": "cobot_joint_6"
        }
        self.get_logger().info("Agência de Notícias Rebranded: Iniciada!")

    def callback(self, msg):
        new_msg = JointState()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = "mycobot_base_link"
        
        for i, name in enumerate(msg.name):
            if name in self.map:
                new_msg.name.append(self.map[name])
                val = msg.position[i]
                # Converte Graus para Radianos se necessário
                if abs(val) > 3.2:
                    val = math.radians(val)
                new_msg.position.append(val)
        
        if new_msg.name:
            self.pub.publish(new_msg)

def main():
    rclpy.init()
    rclpy.spin(JointRebrander())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
