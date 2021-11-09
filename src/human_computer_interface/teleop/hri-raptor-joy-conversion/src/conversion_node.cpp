#include "conversion.hpp"

namespace hri_raptor_joystick_interface {
  HRIRaptorConversion::HRIRaptorConversion() : rclcpp::Node("HRIRaptorConversionNode") {
    this->declare_parameter("input_enable_button");
    input_enable_button_ = this->get_parameter("input_enable_button").as_int();
    this->declare_parameter("output_enable_button");
    output_enable_button_ = this->get_parameter("output_enable_button").as_int();
    this->declare_parameter("input_disable_button");
    input_disable_button_ = this->get_parameter("input_disable_button").as_int();
    this->declare_parameter("output_disable_button");
    output_disable_button_ = this->get_parameter("output_disable_button").as_int();
    this->declare_parameter("input_left_emergency_button");
    input_left_emergency_button_ = this->get_parameter("input_left_emergency_button").as_int();
    this->declare_parameter("output_left_emergency_button");
    output_left_emergency_button_ = this->get_parameter("output_left_emergency_button").as_int();
    this->declare_parameter("input_right_emergency_button");
    input_right_emergency_button_ = this->get_parameter("input_right_emergency_button").as_int();
    this->declare_parameter("output_right_emergency_button");
    output_right_emergency_button_ = this->get_parameter("output_right_emergency_button").as_int();
    this->declare_parameter("input_steering_axis");
    input_steering_axis_ = this->get_parameter("input_steering_axis").as_int();
    this->declare_parameter("output_steering_axis");
    output_steering_axis_ = this->get_parameter("output_steering_axis").as_int();
    this->declare_parameter("reverse_steering");
    reverse_steering_ = this->get_parameter("reverse_steering").as_bool() ? -1 : 1;
    this->declare_parameter("input_throttle_axis");
    input_throttle_axis_ = this->get_parameter("input_throttle_axis").as_int();
    this->declare_parameter("output_throttle_axis");
    output_throttle_axis_ = this->get_parameter("output_throttle_axis").as_int();
    this->declare_parameter("input_brake_axis");
    input_brake_axis_ = this->get_parameter("input_brake_axis").as_int();
    this->declare_parameter("output_brake_axis");
    output_brake_axis_ = this->get_parameter("output_brake_axis").as_int();
    this->declare_parameter("reverse_throttle", false);
    reverse_throttle_ = this->get_parameter("reverse_throttle").as_bool() ? -1 : 1;
    this->declare_parameter("raptor_button_count", 11);
    button_cnt_ = this->get_parameter("raptor_button_count").as_int();
    this->declare_parameter("raptor_axis_count", 8);
    axis_cnt_ = this->get_parameter("raptor_axis_count").as_int();
    this->declare_parameter("throttle_threshold");
    throttle_threshold_ = this->get_parameter("throttle_threshold").as_double();
    this->declare_parameter("brake_threshold");
    brake_threshold_ = this->get_parameter("brake_threshold").as_double();
    this->declare_parameter("throttle_scale");
    throttle_scale_ = this->get_parameter("throttle_scale").as_double();
    this->declare_parameter("steering_scale");
    steering_scale_ = this->get_parameter("steering_scale").as_double();
    this->declare_parameter("brake_scale");
    brake_scale_ = this->get_parameter("brake_scale").as_double();
    this->declare_parameter("throttle_brake_same_axis");
    throttle_brake_same_axis_ = this->get_parameter("throttle_brake_same_axis").as_bool();
    hri_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy_sub", 600,
                      std::bind(&HRIRaptorConversion::joyCallback, this, std::placeholders::_1));
    raptor_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy_pub", 1);
    if (button_cnt_ == 0 or axis_cnt_ == 0) {
      RCLCPP_ERROR(this->get_logger(), "Button Cnt and Axis Cnt can't be 0");
    }
    RCLCPP_INFO(this->get_logger(), "Joy Conversion Node initialized");
  }

  void HRIRaptorConversion::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received Joy message");
    joy_pub_msg_.header = msg->header;
    std::vector<float> converted_axes(axis_cnt_, 0.0);
    std::vector<int> converted_buttons(button_cnt_, 0);
    converted_buttons[output_enable_button_] = msg->buttons[input_enable_button_];
    converted_buttons[output_disable_button_] = msg->buttons[input_disable_button_];
    converted_buttons[output_left_emergency_button_] = msg->buttons[input_left_emergency_button_];
    converted_buttons[output_right_emergency_button_] = msg->buttons[output_right_emergency_button_];
    converted_axes[output_steering_axis_] = reverse_steering_ * msg->axes[input_steering_axis_];

    msg->axes[input_throttle_axis_] *= reverse_throttle_;

    if (throttle_brake_same_axis_) {
      if (msg->axes[input_throttle_axis_] == 0) {
        converted_axes[output_throttle_axis_] = 1.0;
        converted_axes[output_brake_axis_] = 1.0;
      } else if (msg->axes[input_brake_axis_] < brake_threshold_) {
        converted_axes[output_throttle_axis_] = 1.0;
        converted_axes[output_brake_axis_] = (msg->axes[input_throttle_axis_] * 2.0) +1.0;
      } else if (msg->axes[input_throttle_axis_] > throttle_threshold_) {
        converted_axes[output_throttle_axis_] = (msg->axes[input_throttle_axis_] * -2.0) + 1.0;
        converted_axes[output_brake_axis_] = 1.0;
      }
      else {
        converted_axes[output_brake_axis_] = 1.0;
        converted_axes[output_throttle_axis_] = 1.0;
      }
    } else {
      if (msg->axes[input_throttle_axis_] == 0) {
        converted_axes[output_throttle_axis_] = 1.0;
        converted_axes[output_brake_axis_] = 1.0;
      } else if (msg->axes[input_brake_axis_] < brake_threshold_) {
        converted_axes[output_throttle_axis_] = 1.0;
        converted_axes[output_brake_axis_] = (msg->axes[input_throttle_axis_] * 2.0) +1.0;
      } else if (msg->axes[input_throttle_axis_] < throttle_threshold_) {
        converted_axes[output_throttle_axis_] = (msg->axes[input_throttle_axis_] * -2.0) + 1.0;
        converted_axes[output_brake_axis_] = 1.0;
      }
      else {
        converted_axes[output_brake_axis_] = 1.0;
        converted_axes[output_throttle_axis_] = 1.0;
      }
    }
    
    converted_axes[output_brake_axis_] *= brake_scale_;
    converted_axes[output_throttle_axis_] *= throttle_scale_;
    converted_axes[output_steering_axis_] *= steering_scale_;
    joy_pub_msg_.axes = converted_axes;
    joy_pub_msg_.buttons = converted_buttons;
    raptor_joy_pub_->publish(joy_pub_msg_);
    RCLCPP_DEBUG(this->get_logger(), "Published converted Joy message");
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<hri_raptor_joystick_interface::HRIRaptorConversion>());
  rclcpp::shutdown();
  return 0;
}
