#ifndef LGSVL_SENDER_HPP_
#define LGSVL_SENDER_HPP_

#include <rclcpp/rclcpp.hpp>
#include "tai_interface/msg/vehicle_control.hpp"
#include "lgsvl_msgs/msg/vehicle_control_data.hpp"

namespace lgsvl_tai
{
    using lgsvl_msgs::msg::VehicleControlData;
    using std::placeholders::_1;
    using tai_interface::msg::VehicleControl;

    class LGSVLSender : public rclcpp::Node
    {
    public:
        explicit LGSVLSender(const rclcpp::NodeOptions &options);
    private:
        rclcpp::Publisher<VehicleControlData>::SharedPtr lgsvl_cmd_pub_;
        rclcpp::Subscription<VehicleControl>::SharedPtr tai_cmd_sub_;

        void tai_callback(const VehicleControl::SharedPtr tai_cmd);
        float wheel_angle_rate;
    };
}
#endif