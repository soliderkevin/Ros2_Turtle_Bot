#!/usr/bin/env python

import rclpy  # ✅ ROS 2's Python library
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

class PurePursuitController:
    def __init__(self):
        rospy.init_node('pure_pursuit_controller')

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.lookahead_distance = 0.5  # meters
        self.linear_speed = 0.2  # constant forward speed

        self.pose = None

        # Define waypoints [(x1, y1), (x2, y2), ...]
        self.waypoints = [(1, 0), (2, 1), (3, 1), (4, 0)]

        self.goal_reached = False
        self.current_waypoint_index = 0

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def run(self):
        while not rospy.is_shutdown():
            if self.pose is None or self.goal_reached:
                self.rate.sleep()
                continue

            # Get current robot position and orientation
            x = self.pose.position.x
            y = self.pose.position.y

            quaternion = (
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w)
            _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

            # Find lookahead point
            target = self.find_lookahead_point(x, y)
            if target is None:
                rospy.loginfo("Goal reached!")
                self.goal_reached = True
                self.cmd_pub.publish(Twist())  # stop
                continue

            # Compute steering angle
            tx, ty = target
            dx = tx - x
            dy = ty - y

            # Transform target point to robot's coordinate frame
            local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

            # Calculate curvature and angular velocity
            if local_x == 0:
                curvature = 0
            else:
                curvature = (2 * local_y) / (self.lookahead_distance ** 2)

            angular_z = curvature * self.linear_speed

            # Publish Twist command
            cmd = Twist()
            cmd.linear.x = self.linear_speed
            cmd.angular.z = angular_z
            self.cmd_pub.publish(cmd)

            self.rate.sleep()

    def find_lookahead_point(self, x, y):
        for i in range(self.current_waypoint_index, len(self.waypoints)):
            wx, wy = self.waypoints[i]
            dist = math.hypot(wx - x, wy - y)
            if dist >= self.lookahead_distance:
                self.current_waypoint_index = i
                return wx, wy

        # Final goal check
        if self.current_waypoint_index >= len(self.waypoints) - 1:
            dist_to_goal = math.hypot(self.waypoints[-1][0] - x,
                                      self.waypoints[-1][1] - y)
            if dist_to_goal < 0.2:
                return None
            else:
                return self.waypoints[-1]

if __name__ == '__main__':
    try:
        controller = PurePursuitController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
