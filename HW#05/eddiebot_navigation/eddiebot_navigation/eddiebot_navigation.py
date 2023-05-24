import os
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from rclpy.node import Node

BLUE = [np.array([0, 20, 0]), np.array([100, 100, 10])]
RED = [np.array([0, 0, 70]), np.array([20, 20, 200])]
GREEN = [np.array([0, 70, 0]), np.array([20, 200, 20])]
YELLOW = [np.array([0, 70, 70]), np.array([10, 200, 200])]
HEIGHT = 480
WIDTH = 640

class EddiebotNavigation(Node):
    def __init__(self):
        super().__init__('eddiebot_navigation')
        self.camera_subscription = self.create_subscription(Image, '/kinect_rgbd_camera/image', self.camera_listener, 10)

        self._forward = True
        self._target=1
        self._red = False
        self._yellow = False
        self._green = False

        self._ang_vel=0.0

        self._turning=False
        self._count=0

        self.publisher = self.create_publisher(Twist,'/model/eddiebot/cmd_vel',10)
        self.timer = self.create_timer(0.5, self.on_timer)


    def camera_listener(self, msg : Image):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(HEIGHT, WIDTH, -1)
        size = HEIGHT*WIDTH
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        l = img[:,0:320,:]
        r = img[:,320:640,:]

        left = [0,0,0]
        right = [0,0,0]

        left[0] = cv2.countNonZero(cv2.inRange(l, RED[0], RED[1]))
        right[0] = cv2.countNonZero(cv2.inRange(r, RED[0], RED[1]))

        left[1] = cv2.countNonZero(cv2.inRange(l, YELLOW[0], YELLOW[1]))
        right[1] = cv2.countNonZero(cv2.inRange(r, YELLOW[0], YELLOW[1]))

        left[2] = cv2.countNonZero(cv2.inRange(l, GREEN[0], GREEN[1]))
        right[2] = cv2.countNonZero(cv2.inRange(r, GREEN[0], GREEN[1]))


            
        
        if left[self._target]!=0 and right[self._target]==0: # box is in the left half
            self._ang_vel = 10*left[self._target]/size              
        elif left[self._target]==0 and right[self._target]!=0: # box is in the right half
            self._ang_vel = -10*right[self._target]/size              
        elif left[self._target]==0 and right[self._target]==0:  
            self._ang_vel = 0.0           
        elif left[self._target]!=0 and right[self._target]!=0: # box in both halves of the frame
            self._ang_vel = left[self._target]/size - right[self._target]/size  
        # self.get_logger().info(f"RATIO: {self._ang_vel}")


        red=left[0]+right[0]
        green=left[2]+right[2]
        yellow=left[1]+right[1]

        if red > yellow and red > green:
            self._target=0
            if red/size > 0.8:
                self._red = True
                self._yellow = False
                self._green = False
                self._forward = False
        elif yellow > red and yellow > green:
            self._target=1
            if yellow/size > 0.8:
                self._yellow = True
                self._green = False
                self._forward = False
                self._red = False
        elif green > red and green > yellow:
            self._target=2
            if green/size > 0.8:
                self._green = True
                self._forward = False
                self._red = False
                self._yellow = False
        else:
            if not self._turning:
                self._forward = True
                self._red = False
                self._yellow = False
                self._green = False


    def on_timer(self):
        twist = Twist()
        
        if self._red:
            self._turning=True
            twist.linear.x = 0.0
            if self._count >= 45:
                twist.angular.z = -0.5
                self._red = False 
                self._count=0
                self._turning=False
                self._forward = True
            else:
                twist.angular.z = 0.5
                self._count+=1  

        elif self._yellow:
            self._turning=True
            twist.linear.x = 0.0
            if self._count >= 45:
                twist.angular.z = 0.5
                self._yellow = False 
                self._count = 0
                self._turning=False
                self._forward = True
            else:
                twist.angular.z = -0.5
                self._count+=1   

        elif self._green:
            twist.angular.z = 0.5
            twist.linear.x = 0.0

        elif self._forward:
            twist.angular.z = self._ang_vel
            twist.linear.x = 1.0

        self.publisher.publish(twist)



def main():
    rclpy.init()
    node = EddiebotNavigation()
    rclpy.spin(node)
    rclpy.shutdown()