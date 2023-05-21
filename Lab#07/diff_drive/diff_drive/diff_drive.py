from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import time

class DiffDrive(Node):

    def __init__(self):
        super().__init__('diff_drive')

        self.FORWARD = True
        self.publisher = self.create_publisher(Twist,'/cmd_vel',1)
        self.timer = self.create_timer(2.0, self.on_timer)

    def on_timer(self):
        msg = Twist()
        if self.FORWARD:
            msg.angular.z = 0.0
            msg.linear.x = 1.0
            self.FORWARD = False
            self.get_logger().info('self.FORWARD = True')

        else:
            msg.angular.z = 0.0
            msg.linear.x = -1.0
            self.FORWARD = True
            self.get_logger().info('self.FORWARD = False')

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = DiffDrive()
    rclpy.spin(node)
    rclpy.shutdown()

