import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class StartPublisher(Node):
    def __init__(self):
        super().__init__('start_publisher')
        self.publisher_ = self.create_publisher(Bool, '/start', 10)

        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.published = False

    def timer_callback(self):
        if not self.published:
            msg = Bool()
            msg.data = True
            self.publisher_.publish(msg)
            self.get_logger().info('Published start signal: True')
            self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = StartPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
