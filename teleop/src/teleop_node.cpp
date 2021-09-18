#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"
#include "teleop_interfaces/srv/emergency_stop.hpp"


using namespace std::chrono_literals;

class Teleop : public rclcpp::Node {
public:
    Teleop() : Node("teleop_node") {

        // subscribe to joystick and publish to lgsvl
        subscription = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy/joy", 10, std::bind(&Teleop::topic_callback, this, std::placeholders::_1));
        publisher = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/lgsvl/vehicle_control_cmd", 10);
        
        // accept the estop service
        service = this->create_service<teleop_interfaces::srv::EmergencyStop>("estop", 
            std::bind(&Teleop::toggle_estop, this, std::placeholders::_1, std::placeholders::_2));

        // ref: http://docs.ros.org.ros.informatik.uni-freiburg.de/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html
        /*
            Note: Stuff related to changing gears has been commented because its not 
            yet supported for lgsvl :(
            
            ref: https://github.com/lgsvl/simulator/issues/1303
        */

        this->declare_parameter("accel", 5);
        this->declare_parameter("braking", 5);
        this->declare_parameter("wheel", 0);
        this->declare_parameter("estop", 5);
        // this->declare_parameter("drive", 3);
        // this->declare_parameter("parking", 1);
        // this->declare_parameter("reverse", 0);
        // this->declare_parameter("low", 2);
        this->declare_parameter("target_angular_rate", 40);

        this->get_parameter("accel", accel);
        this->get_parameter("braking", braking);
        this->get_parameter("wheel", wheel);
        this->get_parameter("estop", estop);
        // this->get_parameter("drive", drive);
        // this->get_parameter("parking", parking);
        // this->get_parameter("reverse", reverse);
        // this->get_parameter("low", low);
        this->get_parameter("target_angular_rate", target_angular_rate);
    }

private:
    // int drive, parking, reverse, low; // vars for gears
    int accel, braking, wheel, target_angular_rate, estop; // control vars
    bool estop_state = false;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher;
    rclcpp::Service<teleop_interfaces::srv::EmergencyStop>::SharedPtr service;

    void toggle_estop(const std::shared_ptr<teleop_interfaces::srv::EmergencyStop::Request> request,
        std::shared_ptr<teleop_interfaces::srv::EmergencyStop::Response> response) {
        
        this->estop_state = request->set_estop;
        response->estop_state = this->estop_state;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emergency stop %s", this->estop_state ? "enabled" : "disabled");
    }

    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        
        lgsvl_msgs::msg::VehicleControlData vehicle_control;

        if (msg->buttons[estop] || estop_state) {
            vehicle_control.acceleration_pct =  0.0;
            vehicle_control.braking_pct = 1.0;
            vehicle_control.target_wheel_angle = 0.0;
            vehicle_control.target_wheel_angular_rate = target_angular_rate;
            vehicle_control.target_gear = 1; //get_gear(msg); 
        } else {
            vehicle_control.acceleration_pct =  map_linear<double>(1, -1, 0, 1, msg->axes[accel]); 
            vehicle_control.braking_pct = map_linear<double>(1, -1, 0, 1, msg->axes[braking]);
            vehicle_control.target_wheel_angle = map_linear<double>(1, -1, - M_PI/2 , M_PI / 2, msg->axes[wheel]);
            vehicle_control.target_wheel_angular_rate = target_angular_rate;
            vehicle_control.target_gear = 1; //get_gear(msg); 
        }
        
        publisher->publish(vehicle_control);
    }

    template <class T>
    T map_linear(T ip_start, T ip_end, T op_start, T op_end, T val) const {
        return ((val - ip_start)/ (ip_end - ip_start)) * (op_end - op_start) + op_start;
    }

    // int get_gear(const sensor_msgs::msg::Joy::SharedPtr msg) const {
    //     if (msg->buttons[drive]) {
    //         return 1;
    //     } else if (msg->buttons[reverse]) {
    //         return 2;
    //     } else if (msg->buttons[parking]) {
    //         return 3;
    //     } else if (msg->buttons[low]) {
    //         return 4;
    //     }
    //     return 0;
    // }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    return 0;
}