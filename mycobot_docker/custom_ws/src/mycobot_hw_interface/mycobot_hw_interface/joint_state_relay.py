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
        
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        # O robô vai publicar em 'joint_states_raw'
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
            
        self.subscription = self.create_subscription(
            JointState,
            'joint_states_raw',
            self.listener_callback,
            qos)
        
        # O MoveIt vai ler de 'joint_states' (com o carimbo do PC)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.count = 0
        self.get_logger().info('Joint State Relay initialized. Re-stamping hardware messages with PC time.')

    def listener_callback(self, msg):
        self.count += 1
        # Diagnostic logging
        if self.count % 10 == 0:
            self.get_logger().info(f'Relayed {self.count} states. Original stamp: {msg.header.stamp.sec}')
        
        # Override the stamp with PC time to satisfy MoveIt's "recent state" check
        msg.header.stamp = self.get_clock().now().to_msg()
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
