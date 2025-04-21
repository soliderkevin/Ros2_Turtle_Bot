import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
class TurtleMover(Node):
    def __init__(self):
        super().__init__('turtle_move')
        self.publisher_ = self.create_publisher(Twist,'/turtle/cmd_vel',10)
        self.timer = self.create_timer(0.5, self.timer_callback)
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 1.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0

        self.publisher_publish(msg)
        self.get_logger().info("Publishing: linear.x%.2f angular.z=%.2f" % (msg.linear.x, msg.angular.z))



class TurtleWallAvoider(Node):
    def __init__(self):
        super().__init__('turtle_wall_avoider')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.turtle_pose = None
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # 定義牆壁邊界（根據 turtlesim 的標準座標系統）
        self.x_min, self.x_max = 1.0, 10.0
        self.y_min, self.y_max = 1.0, 10.0
        self.safe_distance = 1.0  # 與牆壁的安全距離

    def pose_callback(self, msg):
        self.turtle_pose = msg

    def control_loop(self):
        if self.turtle_pose is None:
            return
        
        msg = Twist()
        near_wall = self.check_near_wall()
        
        if near_wall:
            msg.linear.x = 0.5
            msg.angular.z = 1.0  # 轉彎避開牆壁
            self.get_logger().info('Near wall! Turning...')
        else:
            msg.linear.x = 1.0
            msg.angular.z = 0.0  # 正常前進
        
        self.publisher_.publish(msg)

    def check_near_wall(self):
        """檢查烏龜是否接近牆壁"""
        if (self.turtle_pose.x < self.x_min + self.safe_distance or
            self.turtle_pose.x > self.x_max - self.safe_distance or
            self.turtle_pose.y < self.y_min + self.safe_distance or
            self.turtle_pose.y > self.y_max - self.safe_distance):
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    turtle_wall_avoider = TurtleWallAvoider()
    rclpy.spin(turtle_wall_avoider)
    turtle_wall_avoider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
