#include "lgsvl_tai_interface/lgsvl_sender.hpp"

namespace lgsvl_tai
{
    LGSVLSender::LGSVLSender(const rclcpp::NodeOptions &options)
        : rclcpp::Node("lgsvl_sender_node", options)
    {
        lgsvl_cmd_pub_ = create_publisher<VehicleControlData>("/lgsvl/vehicle_cmd", 1);
        tai_cmd_sub_ = create_subscription<TritonAIRacerControl>("/vehicle_cmd", 1, std::bind(&LGSVLSender::tai_callback, this, _1));
        wheel_angle_rate = declare_parameter<float>("target_wheel_angular_rate", 1.0);
    }

    void LGSVLSender::tai_callback(const TritonAIRacerControl::SharedPtr tai_cmd)
    {
        auto lgsvl_cmd = VehicleControlData();
        lgsvl_cmd.acceleration_pct = tai_cmd->throttle.throttle;
        lgsvl_cmd.braking_pct = tai_cmd->brake.brake;
        lgsvl_cmd.target_wheel_angle = tai_cmd->steering_rad.steer;
        lgsvl_cmd.target_wheel_angular_rate = wheel_angle_rate;
        lgsvl_cmd_pub_->publish(lgsvl_cmd);
    }
}