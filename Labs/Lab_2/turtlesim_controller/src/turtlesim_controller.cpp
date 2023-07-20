#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "turtlesim/msg/pose.hpp"
#include "turtlesim/action/rotate_absolute.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define PI 3.141592653589

enum State
{
  FORWARD,
  BACKWARD,
  TURN,
  STOP,
};

class TurtleControllerNode : public rclcpp::Node
{
public:
  using RotateAbsolute = turtlesim::action::RotateAbsolute;
  using GoalHandleRotateAbsolute = rclcpp_action::ClientGoalHandle<RotateAbsolute>;

    TurtleControllerNode() : Node("turtle_controller")
    {
        this->declare_parameter("stop", false);
        stop_ = this->get_parameter("stop").as_bool();

        window_max=11.0;
        window_min=1e-3;
        count=0;

        twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);

        pose_subscriber = this->create_subscription<turtlesim::msg::Pose>(
              "turtle1/pose", 10, std::bind(&TurtleControllerNode::poseCallback, this, std::placeholders::_1));
        
        control_loop_timer = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&TurtleControllerNode::controlLoop, this));

        if (stop_){
          state = STOP;
        }
        else {
          state = FORWARD;
        }

        rotate_absolute_client = rclcpp_action::create_client<RotateAbsolute>(
          this,"turtle1/rotate_absolute");

    }


private:

    void controlLoop(){
      if (state == FORWARD) {
        if (hitWallCheck()){
          count=0;
          state = BACKWARD;
          backward();
        }
        else {
          forward();
        }
      }
      else if (state == BACKWARD) {
        if (count+1 == 4) {
          state = TURN;
          send_goal();
        }
        else {
          count++;
          backward();
        }
      }
      else if (state == STOP) {
        stop();
      }
    }

    void send_goal(){
      using namespace std::placeholders;      
      auto goal_msg = RotateAbsolute::Goal();
      goal_msg.theta = 2 * PI * (double)rand() / RAND_MAX;

      RCLCPP_INFO(this->get_logger(), "Sending goal...");
      auto send_goal_options = rclcpp_action::Client<RotateAbsolute>::SendGoalOptions();
      send_goal_options.feedback_callback =std::bind(&TurtleControllerNode::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =std::bind(&TurtleControllerNode::result_callback, this, _1);
      
      rotate_absolute_client->async_send_goal(goal_msg, send_goal_options);
    }
    void feedback_callback(GoalHandleRotateAbsolute::SharedPtr, 
    const std::shared_ptr<const RotateAbsolute::Feedback> feedback) {
      RCLCPP_INFO(this->get_logger(),"%f",feedback->remaining);
    }
    void result_callback(const GoalHandleRotateAbsolute::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          state=FORWARD;
          RCLCPP_ERROR(this->get_logger(), "Rotation finished!");

          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      
    }


    void stop(){
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      twist_publisher->publish(twist);
    }

    void forward(){
      twist.linear.x = 1.0;
      twist.angular.z = 0.0;
      twist_publisher->publish(twist);
    }
    void backward(){
      twist.linear.x = -1.0;
      twist.angular.z = 0.0;
      twist_publisher->publish(twist);
    }

    bool hitWallCheck(){
      if (window_max < pose_.y || // hit top wall
        window_min > pose_.y || // hit bottom wall
        window_max < pose_.x || // hit right wall
        window_min > pose_.x) // hit left wall
      {
        return true;
      }
      return false;
    }

//                 SUBSCRIBER
// Pose.msg
// float32 x
// float32 y
// float32 theta
// float32 linear_velocity
// float32 angular_velocity
    void poseCallback(const turtlesim::msg::Pose::SharedPtr pose)
    {
        pose_ = *pose.get();
    }




    turtlesim::msg::Pose pose_;
    geometry_msgs::msg::Twist twist;
    int count;
    double window_max;
    double window_min;

    State state;
    bool stop_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher;

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber;

    rclcpp::TimerBase::SharedPtr control_loop_timer;

    rclcpp_action::Client<RotateAbsolute>::SharedPtr rotate_absolute_client;
};



int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
