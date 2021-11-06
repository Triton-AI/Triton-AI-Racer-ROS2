/**
 * @file js_tai_interface_node.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Convert joystick message to TritonAIRacer control message
 * @version 0.1
 * @date 2021-11-06
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */

#ifndef JS_TAI_INTERFACE__JS_TAI_INTERFACE_NODE_HPP_
#define JS_TAI_INTERFACE__JS_TAI_INTERFACE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "tai_interface/msg/vehicle_control.hpp"

namespace tritonai
{
    namespace interface
    {
        using sensor_msgs::msg::Joy;
        using tai_interface::msg::VehicleControl;

        struct JsConfig
        {
            int64_t axis_stick_steering;
            int64_t axis_trigger_throttle;
            int64_t axis_trigger_brake;

            bool steering_flipped;
            bool throttle_flipped;
            bool brake_flipped;
        };

        class JsTaiInterfaceNode : public rclcpp::Node
        {
        public:
            explicit JsTaiInterfaceNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::Subscription<Joy>::SharedPtr js_sub_;
            rclcpp::Publisher<VehicleControl>::SharedPtr ctl_pub_;
            JsConfig js_config_;

            void jsCallback(const Joy::SharedPtr joy);
            double map_paddel(const double &js_val, const bool &flipped);
            double map_steer(const double &js_val, const bool &flipped);
        };
    }
}

#endif // JS_TAI_INTERFACE__JS_TAI_INTERFACE_NODE_HPP_