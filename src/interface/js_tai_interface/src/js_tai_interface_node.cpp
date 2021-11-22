/**
 * @file js_tai_interface_node.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Convert joystick message to TritonAIRacer control message
 * @version 0.1
 * @date 2021-11-06
 *
 * @copyright Copyright (c) 2021 [Triton AI]
 *
 */
#include <algorithm>
#include "js_tai_interface/js_tai_interface_node.hpp"
namespace tritonai
{
namespace interface
{
using std::placeholders::_1;

JsTaiInterfaceNode::JsTaiInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pilot_node", options)
{
  declare_parameter<int64_t>("axis_stick_steering", 0);
  declare_parameter<int64_t>("axis_trigger_throttle", 1);
  declare_parameter<int64_t>("axis_trigger_brake", 2);
  declare_parameter<bool>("steering_flipped", false);
  declare_parameter<bool>("throttle_flipped", false);
  declare_parameter<bool>("brake_flipped", false);

  js_config_ = JsConfig{
    get_parameter("axis_stick_steering").as_int(),
    get_parameter("axis_trigger_throttle").as_int(),
    get_parameter("axis_trigger_brake").as_int(),
    get_parameter("steering_flipped").as_bool(),
    get_parameter("throttle_flipped").as_bool(),
    get_parameter("brake_flipped").as_bool(),
  };

  ctl_pub_ = create_publisher<VehicleControl>("vehicle_cmd", rclcpp::QoS{1});
  js_sub_ = create_subscription<Joy>(
    "/joy", rclcpp::QoS{1},
    std::bind(&JsTaiInterfaceNode::jsCallback, this, _1));
}

void JsTaiInterfaceNode::jsCallback(const Joy::SharedPtr joy)
{
  auto ctl_msg = VehicleControl();
  ctl_msg.header.stamp = now();
  ctl_msg.throttle = map_paddel(
    joy->axes[js_config_.axis_trigger_throttle],
    js_config_.throttle_flipped);
  ctl_msg.brake = map_paddel(
    joy->axes[js_config_.axis_trigger_brake],
    js_config_.brake_flipped);
  ctl_msg.steering_openloop = map_steer(
    joy->axes[js_config_.axis_stick_steering],
    js_config_.steering_flipped);
  ctl_pub_->publish(ctl_msg);
}

double JsTaiInterfaceNode::map_paddel(const double & js_val, const bool & flipped)
{
  double paddel_val = (js_val * (flipped ? -1.0 : 1.0) + 1) / 2;
  return std::clamp<double>(paddel_val, 0.0, 1.0);
}
double JsTaiInterfaceNode::map_steer(const double & js_val, const bool & flipped)
{
  double steer_val = js_val * (flipped ? -1.0 : 1.0);
  return std::clamp<double>(steer_val, -1.0, 1.0);
}
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tritonai::interface::JsTaiInterfaceNode)
