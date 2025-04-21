import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool

class TurtlePurePursuit(Node):
    def __init__(self):
        super().__init__('turtle_pure_pursuit')

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.start_subscriber = self.create_subscription(Bool, '/start', self.start_callback, 10)

        self.pose = None
        self.start_received = False
        self.reached_target = False

        # Pure Pursuit 的目標點
        self.target_x = 1.5
        self.target_y = 8.5

        # 控制參數
        self.lookahead_distance = 0.5  # 前視距離
        self.linear_speed = 1.0

        self.timer = self.create_timer(1/30.0, self.control_loop)

    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info('✅ Received start signal.')
            self.start_received = True

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if not self.start_received or self.pose is None or self.reached_target:
            return

        dx = self.target_x - self.pose.x
        dy = self.target_y - self.pose.y
        distance = math.hypot(dx, dy)

        if distance < 0.3:
            self.get_logger().info(f'🏁 Target reached at ({self.target_x:.2f}, {self.target_y:.2f})')
            stop_cmd = Twist()
            self.velocity_publisher.publish(stop_cmd)
            self.reached_target = True
            return

        # Pure Pursuit 控制策略
        goal_theta = math.atan2(dy, dx)
        heading_error = goal_theta - self.pose.theta
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))  # 限制在 [-pi, pi]

        curvature = 2 * math.sin(heading_error) / self.lookahead_distance

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.linear_speed * curvature  # v * kappa = omega

        self.velocity_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
