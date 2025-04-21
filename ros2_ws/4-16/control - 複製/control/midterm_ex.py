import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool

class TurtleTargetController(Node):
    def __init__(self):
        super().__init__('turtle_target_controller')

        # 訂閱烏龜的位姿
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # 發布速度指令
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 訂閱啟動訊號
        self.subscription = self.create_subscription(Bool, '/start', self.listener_callback, 10)
        self.start_received = False

        # 目標座標 (你可以修改成你想要的點)
        self.target_x = 1.5
        self.target_y = 8.5

        self.pose = None
        self.reached_target = False  # 新增：抵達目標的旗標

        self.Kp_linear = 1.5
        self.Kp_angular = 4.0

        self.timer = self.create_timer(1/30.0, self.control_loop)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received start command: {msg.data}')
        if msg.data is True:
            self.start_received = True

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.start_received or self.pose is None or self.reached_target:
            return

        # 計算誤差
        error_x = self.target_x - self.pose.x
        error_y = self.target_y - self.pose.y
        distance = math.sqrt(error_x ** 2 + error_y ** 2)
        target_theta = math.atan2(error_y, error_x)
        error_theta = target_theta - self.pose.theta
        error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

        # 到達終點則停止
        if distance < 0.3:
            self.get_logger().info(f'已到達目標點: ({self.target_x:.2f}, {self.target_y:.2f})')
            stop_cmd = Twist()
            self.velocity_publisher.publish(stop_cmd)
            self.reached_target = True
            return

        # 發布控制指令
        cmd_vel = Twist()
        cmd_vel.linear.x = min(self.Kp_linear * distance, 2.0)
        cmd_vel.angular.z = max(min(self.Kp_angular * error_theta, 2.0), -2.0)
        self.velocity_publisher.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTargetController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
