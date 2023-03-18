import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Subscriber(Node):
    def __init__(self):
        super().__init__('subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        #self.subscription  # prevent unused variable warning
        self.i=0
        self.text=""

    def listener_callback(self, msg):
        xorKey = "9812762770"
        if (msg.data != "*"):
            xor =  ord(msg.data) ^ ord(xorKey[self.i])
            msg.data = chr(xor)
            self.text+=chr(xor)
            self.get_logger().info('I heard: "%s"' % msg.data)
            self.i+=1
        else:
            self.get_logger().info('Message is: "%s"' % self.text)
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()