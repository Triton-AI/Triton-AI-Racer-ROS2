#include <functional>
#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node{
public:
  MinimalPublisher(): Node("minimal_publisher"), count_(0){
    publisher_steer = this->create_publisher<std_msgs::msg::String>("steer", 10);
    publisher_throttle = this->create_publisher<std_msgs::msg::String>("throttle", 10);
    publisher_brake = this->create_publisher<std_msgs::msg::String>("brake", 10);
    //timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }
  void publisher_loop(){
    while(true){
    	auto message = std_msgs::msg::String();
    	message.data = "Hello, world! " + std::to_string(count_++);
    	RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    	publisher_throttle->publish(message);
        std::this_thread::sleep_for(std::chrono::nanoseconds(100000000));
    }
  }
private:
  //rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_steer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_throttle;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_brake;
  size_t count_;
};

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(): Node("minimal_subscriber"){
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "throttle", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }
  virtual void topic_callback(const std_msgs::msg::String::ConstSharedPtr msg) const{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class pilot{
public:
	pilot(MinimalPublisher* the_pub_ptr=nullptr, 
			std::shared_ptr<MinimalSubscriber> the_sub_ptr=nullptr){
		this->the_servant_talker = the_pub_ptr;
		this->the_servant_listener = the_sub_ptr;
	}
	pilot(const pilot &a_pilot){}
	pilot& operator=(const pilot &a_pilot){return *this;}
	~pilot(){}
	void process_rcs_message(std::string the_rcs_message){
		// will process the rcs message
		return;
	}
private:
	MinimalPublisher* the_servant_talker;
	std::shared_ptr<MinimalSubscriber> the_servant_listener;
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  MinimalPublisher the_publisher;
  std::thread t1(&MinimalPublisher::publisher_loop, &the_publisher);

  //rclcpp::spin(std::make_shared<MinimalSubscriber>());
  std::shared_ptr<MinimalSubscriber> the_subscriber_ptr = std::make_shared<MinimalSubscriber>();

  pilot the_pilot(&the_publisher, the_subscriber_ptr);

  rclcpp::spin(the_subscriber_ptr);
  std::cout << "no one can block me!" << std::endl; 
  t1.join();
  rclcpp::shutdown();
  return 0;
}

