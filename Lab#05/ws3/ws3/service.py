from copy import deepcopy
from typing import Optional, Tuple

from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.task import Future
from std_srvs.srv import Trigger


class MyService:
    """
    Python interface for MoveIt 2 Servo that enables real-time control in Cartesian Space.
    This implementation is just a thin wrapper around TwistStamped message publisher.
    """

    def __init__(
        self,
        node: Node,
        frame_id: str,
        linear_speed: float = 1.0,
        angular_speed: float = 1.0,
        callback_group: Optional[CallbackGroup] = None,
    ):
        """
        Construct an instance of `MyService` interface.
          - `node` - ROS 2 node that this interface is attached to
          - `frame_id` - Reference frame in which to publish command messages
          - `linear_speed` - Factor that can be used to scale all input linear twist commands
          - `angular_speed` - Factor that can be used to scale all input angular twist commands
          - `enable_at_init` - Flag that enables initialisation of MoveIt 2 Servo during initialisation
                               Otherwise, `MyService.enable()` must be called explicitly
          - `callback_group` - Optional callback group to use for ROS 2 communication (topics/services/actions)
        """

        self._node = node

        # Create publisher
        self.__twist_pub = self._node.create_publisher(
            msg_type=TwistStamped,
            topic="delta_twist_cmds",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_ALL,
            ),
            callback_group=callback_group,
        )

        # Create service clients
        self.__start_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/start_servo",
            callback_group=callback_group,
        )
        self.__stop_service = self._node.create_client(
            srv_type=Trigger,
            srv_name="/servo_node/stop_servo",
            callback_group=callback_group,
        )

        self.__trigger_req = Trigger.Request()
        
        # Initialize message based on passed arguments
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = frame_id
        self.__twist_msg.twist.linear.x = linear_speed
        self.__twist_msg.twist.linear.y = linear_speed
        self.__twist_msg.twist.linear.z = linear_speed
        self.__twist_msg.twist.angular.x = angular_speed
        self.__twist_msg.twist.angular.y = angular_speed
        self.__twist_msg.twist.angular.z = angular_speed

        while not self.__start_service.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn(f"Service '{self.__start_service.srv_name}' is not yet available...")
        start_service_future = self.__start_service.call_async(self.__trigger_req)
        start_service_future.add_done_callback(self.__enable_done_callback)

    def __del__(self):
        """
        Try to stop MoveIt 2 Servo during destruction.
        """
        try:
            if self.is_enabled:
                self.__stop_service.call_async(self.__trigger_req)
        except:
            pass

    def __call__(self, linear: Tuple[float, float, float] = (0.0, 0.0, 0.0), angular: Tuple[float, float, float] = (0.0, 0.0, 0.0),):
        """
        Apply linear and angular twist using MoveIt 2 Servo.
        Input is scaled by `linear_speed` and `angular_speed`, respectively.
        """
        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self._node.get_clock().now().to_msg()
        twist_msg.twist.linear.x *= linear[0]
        twist_msg.twist.linear.y *= linear[1]
        twist_msg.twist.linear.z *= linear[2]
        twist_msg.twist.angular.x *= angular[0]
        twist_msg.twist.angular.y *= angular[1]
        twist_msg.twist.angular.z *= angular[2]
        self.__twist_pub.publish(twist_msg)


    def __enable_done_callback(self, future: Future):
        result: Trigger.Response = future.result()
        if not result.success:
            self._node.get_logger().error(f"MoveIt Servo could not be enabled. ({result.message})")
        self.__is_enabled = result.success

    @property
    def frame_id(self) -> str:
        return self.__twist_msg.header.frame_id

    @frame_id.setter
    def frame_id(self, value: str):
        self.__twist_msg.header.frame_id = value

    @property
    def linear_speed(self) -> float:
        return self.__twist_msg.twist.linear.x

    @linear_speed.setter
    def linear_speed(self, value: float):
        self.__twist_msg.twist.linear.x = value
        self.__twist_msg.twist.linear.y = value
        self.__twist_msg.twist.linear.z = value

    @property
    def angular_speed(self) -> float:
        return self.__twist_msg.twist.angular.x

    @angular_speed.setter
    def angular_speed(self, value: float):
        self.__twist_msg.twist.angular.x = value
        self.__twist_msg.twist.angular.y = value
        self.__twist_msg.twist.angular.z = value