import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool
class TurtleHexagonController(Node):
    def __init__(self):
        super().__init__('turtle_hexagon_controller')

        # 訂閱烏龜的位置信息
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscription = self.create_subscription(
            Bool,
            '/start',
            self.listener_callback
            ,10)

        self.start_received = False
        
        # timer_period = 0.1
        # self.timer = self.create_timer(timer_period, self.timer_callback)

        # 發布速度指令
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 設定六邊形參數
        self.center_x = 5.5  # 六邊形中心 x 座標
        self.center_y = 5.5  # 六邊形中心 y 座標
        self.radius = 3.0  # 半徑

        # 計算六邊形 6 個頂點
        self.target_points = self.calculate_hexagon_points()
        self.current_target_index = 0  # 當前目標索引

        # P 控制器增益
        self.Kp_linear = 1.5  # 線速度增益
        self.Kp_angular = 4.0  # 角速度增益

        self.pose = None  # 當前位姿

        # # 設定週期控制
        # self.timer = self.create_timer(0.1, self.control_loop)


    def listener_callback(self,msg):
        self.get_logger().info(f"received start command: (msg,data)")
        if msg.data is True:
            self.start_received = True

    def calculate_hexagon_points(self):
        """計算六邊形的 6 個頂點座標"""
        points = []
        for i in range(6):
            angle = math.radians(60 * i)  # 每個頂點間隔 60 度
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            points.append((x, y))
        return points

    def pose_callback(self, msg):
        """更新烏龜的當前位姿"""
        self.pose = msg

    def control_loop(self):
        if self.start_received:

            """主控制迴圈"""
            if self.pose is None:
                return

            # 取得當前目標點
            target_x, target_y = self.target_points[self.current_target_index]

            # 計算誤差
            error_x = target_x - self.pose.x
            error_y = target_y - self.pose.y
            distance = math.sqrt(error_x**2 + error_y**2)  # 目標距離
            target_theta = math.atan2(error_y, error_x)  # 目標角度
            error_theta = target_theta - self.pose.theta  # 角度誤差

            # 視角範圍修正 (-pi 到 pi)
            error_theta = math.atan2(math.sin(error_theta), math.cos(error_theta))

            # P 控制器輸出
            linear_velocity = self.Kp_linear * distance  # 線速度
            angular_velocity = self.Kp_angular * error_theta  # 角速度

            # 設置速度指令
            cmd_vel = Twist()
            cmd_vel.linear.x = min(linear_velocity, 2.0)  # 限制最大速度
            cmd_vel.angular.z = max(min(angular_velocity, 2.0), -2.0)  # 限制角速度
            self.velocity_publisher.publish(cmd_vel)

            # 判斷是否到達目標點
            if distance < 0.3:
                self.get_logger().info(f'已到達六邊形頂點 {self.current_target_index + 1}: ({target_x:.2f}, {target_y:.2f})')

                # 切換到下一個頂點
                self.current_target_index = (self.current_target_index + 1) % 6  # 循環六邊形頂點

def main(args=None):
    rclpy.init(args=args)
    node = TurtleHexagonController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()