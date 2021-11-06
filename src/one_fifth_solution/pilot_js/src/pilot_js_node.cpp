#include "pilot_js/pilot_js_node.hpp"

#include <sstream>
#include <string>
#include <algorithm>
#include <math.h>

namespace pilot_js
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    class PilotNode::GearBox
    {
    public:
        GearBox()
        {
            gears = std::vector<uint8_t>{GEAR_PARKING, GEAR_REVERSE, GEAR_NEUTRAL, GEAR_DRIVE};
            gear_names = std::vector<std::string>{"Parking", "Reverse", "Neutral", "Drive"};
        }

        uint8_t Upshift()
        {
            if (current_gear_idx < gears.size() - 1)
            {
                current_gear_idx++;
            }
            return gears[current_gear_idx];
        }

        uint8_t Downshift()
        {
            if (current_gear_idx > 0)
            {
                current_gear_idx--;
            }
            return gears[current_gear_idx];
        }

        std::string GetGearName(const uint8_t &gear)
        {
            return gear_names[std::find(gears.begin(), gears.end(), gear) - gears.begin()];
        }

        uint8_t GEAR_NEUTRAL = 0;
        uint8_t GEAR_DRIVE = 1;
        uint8_t GEAR_REVERSE = 2;
        uint8_t GEAR_PARKING = 3;
        uint8_t GEAR_LOW = 4;
        std::vector<uint8_t> gears;
        std::vector<std::string> gear_names;
        uint32_t current_gear_idx = 3;
    };
    PilotNode::PilotNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("pilot_js_node", options), js_upshift_value(0), js_downshift_value(0), target_gear(1),
          gear_box_(new GearBox())
    {
        // Params
        js_steering_axis = declare_parameter("js_steering_axis").get<uint32_t>();
        js_throttle_axis = declare_parameter("js_throttle_axis").get<uint32_t>();
        js_upshift_button = declare_parameter("js_upshift_button").get<uint32_t>();
        js_downshift_button = declare_parameter("js_downshift_button").get<uint32_t>();

        js_throttle_flipped = declare_parameter("js_throttle_flipped").get<bool>();
        js_steering_flipped = declare_parameter("js_steering_flipped").get<bool>();

        max_wheel_angle_rad = declare_parameter("max_wheel_angle_rad").get<double>();
        wheel_angle_change_rate = declare_parameter("wheel_angle_change_rate").get<double>();

        // Pubs N' Subs
        ctrl_pub_ = create_publisher<VehicleControlData>("/lgsvl/vehicle_control_cmd", rclcpp::QoS{1});
        js_sub_ = create_subscription<Joy>("/joy/joy", rclcpp::QoS{5},
                                           std::bind(&PilotNode::jsCallback, this, _1));
    }

    void PilotNode::jsCallback(const Joy::SharedPtr joy)
    {
        auto throttle = joy->axes[js_throttle_axis];
        auto steering = joy->axes[js_steering_axis];
        auto new_upshift_value = joy->buttons[js_upshift_button];
        auto new_downshift_value = joy->buttons[js_downshift_button];

        auto ctrl_msg = VehicleControlData();
        //Steering, braking and throttle
        auto thr = calcThrottle(throttle);
        if (thr > 0)
        {
            ctrl_msg.acceleration_pct = calcThrottle(throttle);
        }
        else
        {
            ctrl_msg.braking_pct = -thr;
        }
        ctrl_msg.target_wheel_angle = calcSteering(steering);
        ctrl_msg.target_wheel_angular_rate = wheel_angle_change_rate;

        //Gears
        if (new_upshift_value != js_upshift_value)
        {
            target_gear = gear_box_->Upshift();
            RCLCPP_INFO(get_logger(), "Upshift to " + gear_box_->GetGearName(target_gear));
        }
        if (new_downshift_value != js_downshift_value)
        {
            target_gear = gear_box_->Downshift();
            RCLCPP_INFO(get_logger(), "Downshift to " + gear_box_->GetGearName(target_gear));
        }
        js_upshift_value = new_upshift_value;
        js_downshift_value = new_downshift_value;
        ctrl_msg.target_gear = target_gear;

        ctrl_pub_->publish(ctrl_msg);
    }

    double PilotNode::calcThrottle(const double &raw_throttle)
    {
        double thr = clip(raw_throttle, -1.0, 1.0);
        return js_throttle_flipped ? -thr : thr;
    }
    double PilotNode::calcSteering(const double &raw_steering)
    {
        double str = clip(raw_steering, -1.0, 1.0) * max_wheel_angle_rad;

        // None linear activation
        str = -sqrt(1 - pow(str, 2)) + 1;
        if (raw_steering < 0)
        {
            str *= -1;
        }
        return js_steering_flipped ? -str : str;
    }

}
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pilot_js::PilotNode)
