#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{

public:
  Publisher(): Node("publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    std::string text = "SEND_HELP!*";
    std:: string xorKey = "9812762770";
    

    if ((count_ + 1) == 10){
      auto message = std_msgs::msg::String();
      message.data = "*";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      count_=0;
    }

    else{
      auto message = std_msgs::msg::String();
      
      char original = text[count_];
      char key = xorKey[count_];
      char end;
      end = original ^ key;
      message.data = end%255;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      count_++;
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
