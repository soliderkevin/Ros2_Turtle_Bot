import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Bool

class TurtlePurePursuitController(Node):
    def __init__(self):
        super().__init__('turtle_pure_pursuit_controller')

        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.subscription = self.create_subscription(Bool, '/start', self.listener_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.start_received = False
        self.pose = None

        # Parameters
        self.center_x = 5.5
        self.center_y = 5.5
        self.radius = 3.0
        self.lookahead_distance = 0.8
        self.linear_speed = 2.0  # constant forward speed
        self.goal_tolerance = 0.3

        self.target_points = self.calculate_hexagon_points()
        self.current_target_index = 0

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def listener_callback(self, msg):
        self.get_logger().info(f"Received start command: {msg.data}")
        self.start_received = msg.data

    def pose_callback(self, msg):
        self.pose = msg

    def calculate_hexagon_points(self):
        points = []
        for i in range(6):
            angle = math.radians(60 * i)
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            points.append((x, y))
        return points

    def control_loop(self):
        if not self.start_received or self.pose is None:
            return

        # Current pose
        x = self.pose.x
        y = self.pose.y
        theta = self.pose.theta

        # Get lookahead target
        target = self.find_lookahead_point(x, y)
        if target is None:
            self.get_logger().info("Goal reached. Stopping.")
            self.velocity_publisher.publish(Twist())
            return

        target_x, target_y = target

        # Transform target point to robot frame
        dx = target_x - x
        dy = target_y - y
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # Calculate curvature and angular velocity
        if local_x == 0:
            curvature = 0.0
        else:
            curvature = (2 * local_y) / (self.lookahead_distance ** 2)

        angular_z = curvature * self.linear_speed

        # Publish command
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = max(min(angular_z, 2.0), -2.0)
        self.velocity_publisher.publish(cmd)

        # Check if close enough to current waypoint
        dist = math.hypot(dx, dy)
        if dist < self.goal_tolerance:
            self.get_logger().info(f'Reached hex point {self.current_target_index + 1}: ({target_x:.2f}, {target_y:.2f})')
            self.current_target_index = (self.current_target_index + 1) % len(self.target_points)

    def find_lookahead_point(self, x, y):
        for i in range(self.current_target_index, len(self.target_points)):
            tx, ty = self.target_points[i]
            dist = math.hypot(tx - x, ty - y)
            if dist >= self.lookahead_distance:
                self.current_target_index = i
                return tx, ty

        # If all targets are behind or too close, return final target
        return self.target_points[self.current_target_index]


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
