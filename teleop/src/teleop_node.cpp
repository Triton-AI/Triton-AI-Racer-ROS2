#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class Teleop : public rclcpp::Node {
public:
    Teleop() : Node("teleop_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy/joy", 10, std::bind(&Teleop::topic_callback, this, _1));
        publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("/lgsvl/vehicle_control_cmd", 10);

        // ref: http://docs.ros.org.ros.informatik.uni-freiburg.de/en/galactic/Tutorials/Using-Parameters-In-A-Class-CPP.html
        this->declare_parameter("accel", 5);
        this->declare_parameter("braking", 5);
        this->declare_parameter("wheel", 0);
        this->declare_parameter("drive", 3);
        this->declare_parameter("estop", 0);
        this->declare_parameter("parking", 1);
        this->declare_parameter("reverse", 0);
        this->declare_parameter("low", 2);
        this->declare_parameter("target_angular_rate", 40);

        this->get_parameter("accel", accel);
        this->get_parameter("braking", braking);
        this->get_parameter("wheel", wheel);
        this->get_parameter("drive", drive);
        this->get_parameter("parking", parking);
        this->get_parameter("reverse", reverse);
        this->get_parameter("low", low);
        this->get_parameter("target_angular_rate", target_angular_rate);
    }

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        
        lgsvl_msgs::msg::VehicleControlData vehicle_control;
        vehicle_control.acceleration_pct =  map_linear<double>(1, -1, 0, 1, msg->axes[accel]); 
        vehicle_control.braking_pct = map_linear<double>(1, -1, 0, 1, msg->axes[braking]);
        vehicle_control.target_wheel_angle = map_linear<double>(1, -1, - M_PI/2 , M_PI / 2, msg->axes[wheel]);
        vehicle_control.target_wheel_angular_rate = target_angular_rate;
        vehicle_control.target_gear = get_gear(msg);
        publisher_->publish(vehicle_control);
    }

    template <class T>
    T map_linear(T ip_start, T ip_end, T op_start, T op_end, T val) const {
        return ((val - ip_start)/ (ip_end - ip_start)) * (op_end - op_start) + op_start;
    }

    int get_gear(const sensor_msgs::msg::Joy::SharedPtr msg) const {
        if (msg->buttons[drive]) {
            return 1;
        } else if (msg->buttons[reverse]) {
            return 2;
        } else if (msg->buttons[parking]) {
            return 3;
        } else if (msg->buttons[low]) {
            return 4;
        }
        return 0;
    }

    int accel, braking, wheel, drive, estop, parking, reverse, low, target_angular_rate;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
    return 0;
}