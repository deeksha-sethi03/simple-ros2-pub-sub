import rclpy # A collection of functions that allows users to interact with ROS2 using Python
from rclpy.node import Node # A class that allows users to create ROS2 nodes
from std_msgs.msg import String # Provides the basic message types for ROS2 (e.g. String, Int32, Float32, etc.)

class SimplePublisher(Node):
    def __init__(self):
        # Initialises the node with the name 'simple_publisher'
        super().__init__('simple_publisher') 
        # Creates a publisher that publishes message of String type to a topic named 'topic' and queue size = 10
        self.publisher_ = self.create_publisher(String, 'topic', 10) 
        # Creates a timer that calls the publish_message function every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_message) 
        # Keep track of the number of messages published using the counter
        self.counter = 0

    def publish_message(self):
        # Create a new Strig message object
        msg = String()
        # Updates the message content
        msg.data = f'Hello! We are sending the {self.counter} message now :)'
        # Publishes the message to the 'topic' topic
        self.publisher_.publish(msg)
        # Logs the publishing
        self.get_logger().info(f'Publishing: {self.counter}')
        self.counter += 1

def main(args=None):
    # Initialises the ROS2 System
    rclpy.init(args=args)
    # Creates an instance of the SimplePublisher node
    node = SimplePublisher()
    try:
        # Spins off the node - keeps it running until it is shutdown
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stops the node if the user hits CTRL+C on the keyboard in the terminal
        pass
    # Destroy the node before exiting
    node.destroy_node()
    # Shutsdown the ROS2 System
    rclpy.shutdown()

if __name__ == '__main__':
    main()
