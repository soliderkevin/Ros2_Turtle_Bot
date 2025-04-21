import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtlePControl(Node):
    def __init__(self):
        super().__init__('turtle_p_control')

        # 訂閱烏龜的位置信息
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # 發布速度指令
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 目標點 (固定在烏龜前方)
        self.target_x = 8.0
        self.target_y = 5.5

        # P 控制器增益
        self.Kp_linear = 2.0  # 線速度增益

        self.pose = None  # 當前烏龜位置
        self.timer = self.create_timer(1/30, self.control_loop)  # 100ms 週期執行

    def pose_callback(self, msg):
        """更新烏龜的當前位姿"""
        self.pose = msg

    def control_loop(self):
        """主控制迴圈"""
        if self.pose is None:
            return

        # 計算與目標點的距離
        error_x = self.target_x - self.pose.x
        error_y = self.target_y - self.pose.y
        distance = math.sqrt(error_x**2 + error_y**2)

        if distance<0.05:
            self.get_logger().info(f'到達目標點')

            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0  # 線速度受控於 P 控制器
            cmd_vel.angular.z = 0.0  # 不改變角速度
            self.velocity_publisher.publish(cmd_vel)
        else:
            # P 控制計算線速度
            linear_velocity = self.Kp_linear * distance

            # 限制最大速度
            max_speed = 5.0
            cmd_vel = Twist()
            cmd_vel.linear.x = min(linear_velocity, max_speed)  # 線速度受控於 P 控制器
            cmd_vel.angular.z = 0.0  # 不改變角速度

            self.velocity_publisher.publish(cmd_vel)

            # 顯示當前狀態
            self.get_logger().info(f'距離目標: {distance:.2f}, 設定線速度: {cmd_vel.linear.x:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
