#include "rclcpp/rclcpp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "yinyang_msgs/srv/send_message_in_convo.hpp"
#include "yinyang_msgs/msg/conversation.hpp"
#include "yinyang_msgs/action/opacity.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>

using namespace std::chrono_literals;


class MyYangNode : public rclcpp::Node {


    public:
        MyYangNode() : Node("yang_node")
        {
            this->declare_parameter("shout", false);
            shout_ = this->get_parameter("shout").as_bool();


            state_ = 0;
            isClient_ = false;

            server_ = this->create_service<yinyang_msgs::srv::SendMessageInConvo>(
                "yang_send_message_in_convo",
                std::bind(&MyYangNode::callbackSendMessageInConvoToYinClient, this, std::placeholders::_1, std::placeholders::_2));
            
            RCLCPP_INFO(this->get_logger(), "Server has been started");

            convoPublisher_ = this->create_publisher<yinyang_msgs::msg::Conversation>("conversation", 10);

            control_loop_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(500),
                    std::bind(&MyYangNode::controlLoop, this));


            this->client_ptr_ = rclcpp_action::create_client<yinyang_msgs::action::Opacity>(this,"opacity");


        }

        void send_goal()
        {
            using namespace std::placeholders;

            if (!this->client_ptr_->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
                rclcpp::shutdown();
            }

            auto goal_msg = yinyang_msgs::action::Opacity::Goal();
            goal_msg.request = "Good bye";

            RCLCPP_INFO(this->get_logger(), "Sending goal");

            auto send_goal_options = rclcpp_action::Client<yinyang_msgs::action::Opacity>::SendGoalOptions();
            
            send_goal_options.goal_response_callback =
            std::bind(&MyYangNode::goal_response_callback, this, _1);
            
            send_goal_options.feedback_callback =
            std::bind(&MyYangNode::feedback_callback, this, _1, _2);
            
            send_goal_options.result_callback =
            std::bind(&MyYangNode::result_callback, this, _1);
            
            this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        }
    
    private:
        void controlLoop(){
            if (isClient_){
                if (state_ == 1){
                    if (shout_){
                        std::string str2 ("**Hi Yin, I am Yang the opposite of you.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("Hi Yin, I am Yang the opposite of you.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                }
                if (state_ == 3){
                    if (shout_){
                        std::string str2 ("**Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("Yes, Yin; we ourselves, do not mean anything since we are only employed to express a relation");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
                if (state_ == 5){
                    if (shout_){
                        std::string str2 ("**Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("Precisely, Yin; we are used to describe how things function in relation to each other and to the universe.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
                if (state_ == 7){
                    if (shout_){
                        std::string str2 ("**For what is and what is not beget each other.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("For what is and what is not beget each other.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
                if (state_ == 9){
                    if (shout_){
                        std::string str2 ("**High and low place each other.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("High and low place each other.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
                if (state_ == 11){
                    if (shout_){
                        std::string str2 ("**Before and behind follow each other.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("Before and behind follow each other.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
                if (state_ == 13){
                    if (shout_){
                        std::string str2 ("**And you fade into the darkness.**");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }
                    else {
                        std::string str2 ("And you fade into the darkness.");
                        threads_.push_back(std::thread(std::bind(&MyYangNode::callSendMessageInConvoToYinServer,this, str2)));
                    }

                }
            }
            else {
                RCLCPP_INFO(this->get_logger(),"NOT client");
            }
        }




    // server code (state is EVEN)
        void callbackSendMessageInConvoToYinClient(const yinyang_msgs::srv::SendMessageInConvo::Request::SharedPtr request, 
        const yinyang_msgs::srv::SendMessageInConvo::Response::SharedPtr response) {
            if (!isClient_){
                std::string str = request->message;
                int64_t len = request->length;
                int64_t sum = calculateAsciiSum(str,len);
                response->checksum = sum;
                publishConvo(str,len,sum);                
                state_+=1;
                isClient_=true;
                RCLCPP_INFO(this->get_logger(),"got yin's message - exiting server - state = %d", state_);
            }
        }

        int64_t calculateAsciiSum(std::string str, int64_t len){
            str.erase(std::remove(str.begin(), str.end(), ' '), str.end());
            int64_t sum=0;
            for (int i =0 ; i < len ; i++){
                sum+=(int)str[i];
            }
            return sum;
        }
        void publishConvo(std::string str, int64_t len, int64_t checksum){
            auto msg = yinyang_msgs::msg::Conversation();
            std::string strin ("Yin said: ");
            strin+=str;
            strin+=std::string(" , ");
            strin+=std::to_string(len);
            strin+=std::string(" , ");
            strin+=std::to_string(checksum);
            msg.text = strin;
            convoPublisher_->publish(msg);
        }


    //client code
        void callSendMessageInConvoToYinServer(std::string msg){
            auto client_ = this->create_client<yinyang_msgs::srv::SendMessageInConvo>("yin_send_message_in_convo");
            auto request = std::make_shared<yinyang_msgs::srv::SendMessageInConvo::Request>();
            int64_t k = msg.length();
            request->message = msg;
            request->length = k;
            while (!client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
            }
            auto future = client_->async_send_request(request);
            try
            {
                auto response = future.get();
               
                
                if (state_+ 1 == 15){
                    control_loop_timer_->cancel();
                    send_goal();
                }
                else {
                    state_+=1;
                    isClient_=false;
                }
                
                RCLCPP_INFO(this->get_logger(), "Sent my message, response= %ld ", response->checksum);
                RCLCPP_INFO(this->get_logger(), "waiting for message from yin - state = %d", state_);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
        }






        void goal_response_callback(const rclcpp_action::ClientGoalHandle<yinyang_msgs::action::Opacity>::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        }

        void feedback_callback(
            rclcpp_action::ClientGoalHandle<yinyang_msgs::action::Opacity>::SharedPtr,
            const std::shared_ptr<const yinyang_msgs::action::Opacity::Feedback> feedback)
        {
            RCLCPP_INFO(this->get_logger(), "%ld",feedback->feedback);
        }

        void result_callback(const rclcpp_action::ClientGoalHandle<yinyang_msgs::action::Opacity>::WrappedResult & result)
        {
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
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
            RCLCPP_INFO(this->get_logger(), result.result->result.c_str());
            rclcpp::shutdown();
        }




        int state_;
        bool isClient_;
        bool shout_;
        // auto requestIn = std::make_shared<yinyang_msgs::srv::SendMessageInConvo::Request>();

        rclcpp::Service<yinyang_msgs::srv::SendMessageInConvo>::SharedPtr server_;
        // yinyang_msgs::srv::SendMessageInConvo::Request::SharedPtr requestOut;
        rclcpp::Publisher<yinyang_msgs::msg::Conversation>::SharedPtr convoPublisher_;

        rclcpp::TimerBase::SharedPtr control_loop_timer_;
        std::vector<std::thread> threads_;

        rclcpp_action::Client<yinyang_msgs::action::Opacity>::SharedPtr client_ptr_;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyYangNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}