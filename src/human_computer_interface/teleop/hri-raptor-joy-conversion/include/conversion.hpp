#ifndef HRI_RAPTOR_HPP
#define HRI_RAPTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <vector>

namespace hri_raptor_joystick_interface {
    using std::placeholders::_1;

    class HRIRaptorConversion: public rclcpp::Node {
        public:
            HRIRaptorConversion();

        private:
            size_t button_cnt_;
            size_t axis_cnt_;
            int input_enable_button_;
            int output_enable_button_;
            int input_disable_button_;
            int output_disable_button_;
            int input_left_emergency_button_;
            int output_left_emergency_button_;
            int input_right_emergency_button_;
            int output_right_emergency_button_;
            int right_emergency_button;
            int input_steering_axis_;
            int output_steering_axis_;
            int input_throttle_axis_;
            int output_throttle_axis_;
            int input_brake_axis_;
            int output_brake_axis_;
            int reverse_steering_;
            int reverse_throttle_;
            double throttle_threshold_;
            double brake_threshold_;
            double throttle_scale_;
            double steering_scale_;
            double brake_scale_;
            bool throttle_brake_same_axis_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr hri_joy_sub_;
            rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr raptor_joy_pub_;
            sensor_msgs::msg::Joy joy_pub_msg_;
            
            void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    };
}

#endif
