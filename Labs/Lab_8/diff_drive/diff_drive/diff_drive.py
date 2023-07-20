from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rclpy
from rclpy.node import Node
import time
from math import radians
class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive')

        self.lidar_subscription = self.create_subscription(
            LaserScan, '/lidar', self.lidar_listener, 10)

        self.hit_wall = False
        
        self.publisher = self.create_publisher(Twist,'/cmd_vel',1)
        # self.timer = self.create_timer(2.0, self.on_timer)

    def lidar_listener(self, msg: LaserScan):
        twist = Twist()
        self.hit_wall=False

        for r in msg.ranges:
            if r < 1.0: self.hit_wall=True

        if self.hit_wall:
            twist.angular.z = 0.5
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.5
            twist.angular.z = 0.0

        self.publisher.publish(twist)




    # def on_timer(self):
    #     msg = Twist()
    #     if self.FORWARD:
    #         msg.angular.z = 0.0
    #         msg.linear.x = 1.0
    #         self.FORWARD = False
    #         self.get_logger().info('self.FORWARD = True')

    #     else:
    #         msg.angular.z = 0.0
    #         msg.linear.x = -1.0
    #         self.FORWARD = True
    #         self.get_logger().info('self.FORWARD = False')

    #     self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DiffDrive()
    rclpy.spin(node)
    rclpy.shutdown()

