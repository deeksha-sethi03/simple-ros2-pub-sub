import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePuSub(Node):
    def __init__(self):
        super().__init__('simple_pubsub')
        self.publisher = self.create_publisher(String, '/talker', 10)
        self.timer = self.create_timer(1, self.publisher_callback)
        self.subscriber = self.create_subscription(String, '/talker', self.subscriber_callback, 10)
        self.counter = 0

    def publisher_callback(self):
        msg = String()
        msg.data = f"Publishing: {self.counter}"
        self.publisher.publish(msg)
        self.counter += 1

    def subscriber_callback(self, msg):
        data = msg.data
        self.get_logger().info(f'Subscribed to: {data}')

def main(args=None):
    rclpy.init(args=args)
    node = SimplePuSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
