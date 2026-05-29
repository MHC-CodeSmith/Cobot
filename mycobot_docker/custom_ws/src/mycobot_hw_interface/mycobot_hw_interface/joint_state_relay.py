import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateRelay(Node):
    """
    Relay node to fix timestamp synchronization issues in distributed ROS 2 systems.
    It listens to joint states from the robot (raw) and re-stamps them with the PC's local time.
    """
    def __init__(self):
        super().__init__('joint_state_relay')
        self.joint_names = [
            "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
            "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"
        ]
        self.last_msg = None
        self.seen_raw = False
        
        # O robô (Nano) vai publicar em 'joint_states_raw' via DDS
        # O namespace '/mycobot' é aplicado pelo launch file
        self.subscription = self.create_subscription(
            JointState,
            'joint_states_raw',
            self.listener_callback,
            10)
            
        # O MoveIt vai ler de 'joint_states' (com o carimbo do PC)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.get_logger().info('Joint State Relay DDS Mode: Re-stamping hardware messages with PC time.')

    def listener_callback(self, msg):
        self.last_msg = msg
        if not self.seen_raw:
            self.seen_raw = True
            self.get_logger().info(f'Recebido /joint_states_raw: {list(msg.name)}')

    def publish_joint_states(self):
        msg = self.last_msg if self.last_msg is not None else JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mycobot_base_link'

        if self.last_msg is None:
            msg.name = self.joint_names
            msg.position = [0.0] * 6
            msg.velocity = [0.0] * 6
            msg.effort = [0.0] * 6
            self.get_logger().warn('Sem /joint_states_raw ainda; publicando pose zero temporária.', throttle_duration_sec=5.0)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    relay = JointStateRelay()
    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        pass
    finally:
        relay.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
