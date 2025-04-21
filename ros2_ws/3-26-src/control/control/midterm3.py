import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class StartSubscriber(Node):

    def __init__(self):
        super().__init__('start_subscriber')

        # 訂閱 /start 主題
        self.subscription = self.create_subscription(
            Bool,
            '/start',
            self.listener_callback,
            10)

        # 標記是否收到 True
        self.start_received = False

        # 建立 timer，每 0.5 秒觸發一次 callback
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received start command: {msg.data}")
        self.start_received = msg.data

    def timer_callback(self):
        if self.start_received:
            self.get_logger().info('✅ Start command received! Ready to run controller.')
        else:
            self.get_logger().info('⏳ Waiting for start signal...')


def main(args=None):
    rclpy.init(args=args)

    start_subscriber = StartSubscriber()
    rclpy.spin(start_subscriber)

    start_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
