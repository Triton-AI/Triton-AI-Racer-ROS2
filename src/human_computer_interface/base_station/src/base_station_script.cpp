#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tai_interface/msg/vehicle_mode.hpp"

std::mutex mtx; 

using namespace std::placeholders;
using tai_interface::msg::VehicleOperationMode;

class BaseStation : public rclcpp::Node {
  private:
		rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_control_;
		rclcpp::Publisher<VehicleOperationMode>::SharedPtr pub_vehicle_mode_;

		rclcpp::TimerBase::SharedPtr control_timer_;
		rclcpp::TimerBase::SharedPtr vehicle_mode_timer_;

		uint32_t control_publisher_freq;

		std::string mode = "manual";
		VehicleOperationMode vehicle_mode_cmd;
  public:
	BaseStation() : rclcpp::Node("base_station") {

		declare_parameter("control_publisher_freq", 20);
		get_parameter("control_publisher_freq", control_publisher_freq);

		// Publishers 
		this->pub_control_ = create_publisher<std_msgs::msg::String>("base_station_mode", 1);
		// TODO
		this->pub_vehicle_mode_ = create_publisher<VehicleOperationMode>("vehicle_mode", 1);

		this->control_timer_ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1.0 / control_publisher_freq * 1000)), std::bind(&BaseStation::send_control_callback, this));
		this->vehicle_mode_timer_ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1.0 / control_publisher_freq * 1000)), std::bind(&BaseStation::send_vehicle_mode_callback, this));
	}
	// BaseStation(BaseStation &b) : rclcpp::Node("base_station") {}
	// ~BaseStation() {}
	// BaseStation& operator=(BaseStation &b) {
	// 	return *this;
	// }

	void ask_user_input(){
		while(true){
			//============================================
			int i;
			std::cout << "enter an integer[1-3]:" << std::endl;
			std::cout << "1 manual" << std::endl;
			std::cout << "2 autonomous" << std::endl;
			std::cout << "3 e_stop" << std::endl;
			while (true){
				std::cin >> i;
				if (!std::cin || i < 1 || i > 3){
					std::cout << "Wrong Choice. Enter again " << std::endl;
					std::cin.clear();
					std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
					continue;
				 }
				else break;
			}
			//std::cout << "The integer is : " << i;
			mtx.lock();
			// enum?
			if(i == 1){
				this->mode = "manual";
				this->vehicle_mode_cmd.mode = this->vehicle_mode_cmd.MANUAL;
			}
			else if(i == 2){
				this->mode = "autonomous";
				this->vehicle_mode_cmd.mode = this->vehicle_mode_cmd.AUTONOMOUS;
			}
			else if(i == 3){
				this->mode = "e_stop";
				this->vehicle_mode_cmd.mode = this->vehicle_mode_cmd.EMERGENCY_STOP;
			}
			mtx.unlock();
		}	
	}	

	void send_vehicle_mode_callback() {
		const std::lock_guard<std::mutex> lock(mtx);
		this->pub_vehicle_mode_->publish(this->vehicle_mode_cmd);
	}

	void send_control_callback() { // running repeatedly with the timer set frequency
		mtx.lock();
		auto message = std_msgs::msg::String();
		message.data = this->mode.c_str();
		this->pub_control_->publish(message);
		mtx.unlock();
	}

	void print_local(){ // logging purpose	
		std::cout << "mode: " << this->mode << std::endl;
	}
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  auto ptr_to_object = std::make_shared<BaseStation>();
  std::thread t(&BaseStation::ask_user_input, ptr_to_object);
  rclcpp::spin(ptr_to_object);
  t.join();
  rclcpp::shutdown();
  return 0;
}

