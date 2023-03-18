import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer,  GoalResponse

from yinyang_msgs.srv import SendMessageInConvo
from yinyang_msgs.msg import Conversation
from yinyang_msgs.action import Opacity


class MyYinNode(Node):
    def __init__(self):
        super().__init__("yin_node")
        self.declare_parameter("shout", False)
        self.declare_parameter("opacity", 100)

        self.shout_ = self.get_parameter("shout").value
        self.opacity_ = self.get_parameter("opacity").value

        self.state = 0
        self.is_client = True
        self.sending_once = False

        self.server_ = self.create_service(
            SendMessageInConvo, "yin_send_message_in_convo", self.callback_send_message_in_convo_from_yang_client)

        self.requestOut = SendMessageInConvo.Request()

        self.convo_publisher_ = self.create_publisher(Conversation, "conversation", 10)

        self.control_loop_timer_ = self.create_timer(0.5, self.control_loop)

        self._action_server = ActionServer(
            self,
            Opacity,
            'opacity',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback
            )
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


    def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal...')
            feedback_msg = Opacity.Feedback()
            feedback_msg.feedback = self.opacity_
            while(self.opacity_>=0):
                self.opacity_-=1
                feedback_msg.feedback = self.opacity_
                self.get_logger().info('Feedback: {0}'.format(feedback_msg.feedback))
                goal_handle.publish_feedback(feedback_msg)
                time.sleep(1)
            goal_handle.succeed()
            result = Opacity.Result()
            result.result = "Farewell"
            return result

    def control_loop(self):
        if self.is_client:
            if self.state == 0:
                if self.shout_:
                    str2 = "**I am Yin, some mistake me for an actual material entity but I am more of a concept**"
                else:
                    str2 = "I am Yin, some mistake me for an actual material entity but I am more of a concept"
            elif self.state == 2:
                if self.shout_:
                    str2 = "**Interesting Yang, so one could say, in a philosophical sense, we are two polar elements**"
                else:
                    str2 = "Interesting Yang, so one could say, in a philosophical sense, we are two polar elements"
            elif self.state == 4:
                if self.shout_:
                    str2 = "**We, Yang, are therefore the balancing powers in the universe.**"
                else:
                    str2 = "We, Yang, are therefore the balancing powers in the universe."
            elif self.state == 6:
                if self.shout_:
                    str2 = "**Difficult and easy complete each other.**"
                else:
                    str2 = "Difficult and easy complete each other."
            elif self.state == 8:
                if self.shout_:
                    str2 = "**Long and short show each other.**"
                else:
                    str2 = "Long and short show each other."
            elif self.state == 10:
                if self.shout_:
                    str2 = "**Noise and sound harmonize each other.**"
                else:
                    str2 = "Noise and sound harmonize each other."
            elif self.state == 12:
                if self.shout_:
                    str2 = "**You shine your light.**"
                else:
                    str2 = "You shine your light."

            self.get_logger().info('IS Client, before sending - state = '+ str(self.state))

            self.requestOut.message = str2
            self.requestOut.length = len(str2)
            self.call_send_message_in_convo_to_yang_server()
            self.is_client=False
            self.state+=1
        else:
            self.get_logger().info('NOT Client')



#                           CLIENT (state is EVEN)
    def call_send_message_in_convo_to_yang_server(self):
        client = self.create_client(SendMessageInConvo, "yang_send_message_in_convo")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = client.call_async(self.requestOut)
        future.add_done_callback(self.callback_call_send_message_in_convo_to_yang_server)

    def callback_call_send_message_in_convo_to_yang_server(self, future):
        try:
            response = future.result()       
            self.get_logger().info("Sent my message, response= " + str(response.checksum))
            self.get_logger().info("waiting for message from yang - state = " + str(self.state))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


    def calc_ascii(self,text):
        ascii_sum = 0
        words = text.split()
        for sub in words:
            for ele in sub :
                ascii_sum += (ord(ele))  
        return ascii_sum    

    def publish_convo(self,message,length,checksum):
        msg = Conversation()
        msg.text="Yang said: " + message + " , " + str(length) + " , " + str(checksum)
        self.convo_publisher_.publish(msg)


#                           SERVER (STATE IS ODD)
    def callback_send_message_in_convo_from_yang_client(self, request, response):
        if not self.is_client:
            m = str(request.message)
            response.checksum = self.calc_ascii(str(request.message))
            self.publish_convo(request.message, request.length, response.checksum)
            self.state+=1
            if self.state == 14:
                self.is_client=False
                self.control_loop_timer_.cancel()
            else:
                self.is_client=True
            self.get_logger().info("RECEIVED MEssage - exit server - state= " + str(self.state))
            return response



def main(args=None):
    rclpy.init(args=args)
    node = MyYinNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
