import rclpy
from math import cos, sin
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from ws3 import MyService

def main():
    rclpy.init()

    # Create node for this example
    node = Node("my_node")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 Servo interface
    my_service = MyService(
        node=node,
        frame_id = "6RL0",
        callback_group=callback_group,
    )

    def service_circular_motion():
        """Move in a circular motion using Servo"""

        now_sec = node.get_clock().now().nanoseconds * 1e-9
        my_service(linear=(sin(now_sec), cos(now_sec), 0.0), angular=(0.0, 0.0, 0.0))

    # Create timer for moving in a circular motion
    node.create_timer(0.2, service_circular_motion)

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main() 