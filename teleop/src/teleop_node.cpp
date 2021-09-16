#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

using std::placeholders::_1;

using namespace std::chrono_literals;

class Teleop : public rclcpp::Node
{
  public:
    Teleop()
    : Node("teleop")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Teleop::topic_callback, this, _1));
      publisher_ = this->create_publisher<lgsvl_msgs::msg::VehicleControlData>("vehicle_cmd", 10);
    }


  private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      lgsvl_msgs::msg::VehicleControlData vehicle_control;
      vehicle_control.acceleration_pct = 0.5;
      vehicle_control.braking_pct = 0.5;
      vehicle_control.target_wheel_angle = msg->axes[0] * -1.57;
      vehicle_control.target_wheel_angular_rate = msg->axes[5] < 0 ? 0.5 + (std::abs(msg->axes[5]) / 2) : std::abs(msg->axes[5] - 1) / 2;
      vehicle_control.target_gear = 1;
      publisher_->publish(vehicle_control); 
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Teleop>());
  rclcpp::shutdown();
  return 0;
}