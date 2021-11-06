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
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/string.hpp"

#include "tai_interface/msg/vehicle_control.hpp"
#include "tai_interface/msg/race_state.hpp"
#include "tai_interface/srv/e_stop.hpp"

std::mutex mtx;

using namespace std::placeholders;
using tai_interface::msg::VehicleControl;
using tai_interface::srv::EStop;
using tai_interface::msg::RaceState;

using namespace std::placeholders;

class Pilot : public rclcpp::Node {
  private:
	rclcpp::Publisher<VehicleControl>::SharedPtr pub_control_;

	rclcpp::Subscription<RaceState>::SharedPtr sub_race_state_;
	rclcpp::Subscription<VehicleControl>::SharedPtr sub_manual_vehicle_cmd_;
	rclcpp::Subscription<VehicleControl>::SharedPtr sub_autonomous_vehicle_cmd_;

	rclcpp::Service<EStop>::SharedPtr srv_estop_;

	rclcpp::TimerBase::SharedPtr control_timer_;

	uint32_t control_publisher_freq;
  public:
	std_msgs::msg::UInt16 throttle_cmd;
    std_msgs::msg::UInt16 brake_cmd;
    std_msgs::msg::Int16 steer_cmd;

	VehicleControl  control_cmd;

	Pilot() : rclcpp::Node("pilot") {
		this->throttle_cmd.data = 0;
		this->brake_cmd.data = 0;
		this->steer_cmd.data = 0;
		declare_parameter("control_publisher_freq", 20);
		get_parameter("control_publisher_freq", control_publisher_freq);

		// Publishers 
		this->pub_control_ = create_publisher<VehicleControl>("vehicle_cmd", 1);
		
		// Subscribers
		this->sub_race_state_ = this->create_subscription<RaceState>("/rcs_state", 10, std::bind(&Pilot::receiveRCSState, this, std::placeholders::_1));
		this->sub_manual_vehicle_cmd_ = this->create_subscription<VehicleControl>("manual_vehicle_cmd", 10, std::bind(&Pilot::receiveManual, this, std::placeholders::_1));
		this->sub_autonomous_vehicle_cmd_ = this->create_subscription<VehicleControl>("autonomous_vehicle_cmd", 10, std::bind(&Pilot::receiveAutonomous, this, std::placeholders::_1));

		//Service 
		//srv_estop_ = create_service(EStop, std::bind)
		// Callback timers
		this->control_timer_ = this->create_wall_timer(std::chrono::milliseconds((uint32_t)(1.0 / control_publisher_freq * 1000)), std::bind(&Pilot::send_control_callback, this));

	}
	// Pilot(Pilot &p) : rclcpp::Node("pilot") {}
	// ~Pilot() {}
	// Pilot& operator=(Pilot &p) {
	// 	return *this;
	// }
	

	void receiveRCSState(const RaceState msg) {
		std::cout << "rcs stage recieved: " << msg.stage.stage << std::endl;
		std::cout << "rcs flag recieved: " << msg.flag.race_flag << std::endl;

		if(msg.flag.race_flag == msg.flag.EMERGENCY_STOP) {
			// RED_RED
			std::cout << "rcs: RED_RED" << std::endl;
		}
		else if(msg.flag.race_flag == msg.flag.RED) {
			// RED_FLAG
			std::cout << "rcs: RED_FLAG" << std::endl;
		}
		else if(msg.stage.stage == msg.stage.IN_GARAGE) {
			// IN_GARAGE
			std::cout << "rcs: IN_GARAGE" << std::endl;
		}
		else if(msg.stage.stage == msg.stage.ON_GRID) {
			// GRID ACTIVE
			std::cout << "rcs: GRID_ACTIVE" << std::endl;
		}
		else if(msg.stage.stage == msg.stage.IN_RACE) {
			// GREEN_GREEN
			std::cout << "rcs: GREEN_GREEN" << std::endl;
		}
		
	}

	void receiveManual(const VehicleControl msg) {
		mtx.lock();
		this->control_cmd = msg;
		mtx.unlock();
	}  

	void receiveAutonomous(const VehicleControl msg) {
		mtx.lock();
		this->control_cmd = msg;
		mtx.unlock();
	}


	void send_control_callback() { // running repeatedly with the timer set frequency
		mtx.lock();
		this->pub_control_->publish(this->control_cmd);
		mtx.unlock();
	}

	void print_local(){ // logging purpose	
		std::cout << "throttle: " << this->throttle_cmd.data << std::endl;
		std::cout << "brake: " << this->brake_cmd.data << std::endl;
		std::cout << "steer: " << this->steer_cmd.data << std::endl;
	}
};

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pilot>());
  rclcpp::shutdown();
  return 0;
}

