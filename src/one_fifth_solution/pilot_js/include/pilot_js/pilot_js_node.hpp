#ifndef PILOT_NODE_HPP
#define PILOT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>


#include <map>
#include <string>

namespace pilot_js{
    using std_msgs::msg::Float64;
    using sensor_msgs::msg::Joy;
    using std_msgs::msg::String;
    using vesc_msgs::msg::VescState;

    class PilotNode: public rclcpp::Node
    {
    public:
        explicit PilotNode(const rclcpp::NodeOptions & options);

    private:
    // Params
        bool js_throttle_flipped;
        bool js_steering_flipped;

        uint32_t js_steering_axis;
        uint32_t js_throttle_axis;
        uint32_t js_upshift_button;
        uint32_t js_downshift_button;

        double max_wheel_angle_rad;
        double wheel_angle_change_rate;

        int js_upshift_value;
        int js_downshift_value;
        u_int8_t target_gear;
    // Pub N' Sub
        rclcpp::Publisher<VehicleControlData>::SharedPtr ctrl_pub_;
        rclcpp::Subscription<Joy>::SharedPtr js_sub_;
        void jsCallback(const Joy:: SharedPtr joy);


    // Utils
        template <typename T>
        T clip(const T& n, const T& lower, const T& upper) 
        { return std::max(lower, std::min(n, upper)); }

        double calcThrottle(const double & raw_throttle);
        double calcSteering(const double & raw_steering);
        void emergencyStop();
        void cycleMode();
        class GearBox;
        std::unique_ptr<GearBox> gear_box_;
    };
}








#endif  // PILOT_NODE_HPP
