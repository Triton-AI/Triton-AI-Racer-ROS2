#ifndef raptor_tai_interface_HPP_
#define raptor_tai_interface_HPP_

#include <rclcpp/rclcpp.hpp>
#include "tai_interface/msg/vehicle_control.hpp"

#include "raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp"
#include "raptor_dbw_msgs/msg/brake_cmd.hpp"
#include "raptor_dbw_msgs/msg/steering_cmd.hpp"

#include <mutex>

namespace raptor_tai
{
    using raptor_dbw_msgs::msg::AcceleratorPedalCmd;
    using raptor_dbw_msgs::msg::BrakeCmd;
    using raptor_dbw_msgs::msg::SteeringCmd;
    using tai_interface::msg::VehicleControl;

    class RaptorTaiInterface : public rclcpp::Node
    {
    public:
        explicit RaptorTaiInterface(const rclcpp::NodeOptions &options);

    private:
        rclcpp::Publisher<VehicleControl>::SharedPtr tai_cmd_pub_;
        rclcpp::Publisher<AcceleratorPedalCmd>::SharedPtr raptor_throttle_pub_;
        rclcpp::Publisher<BrakeCmd>::SharedPtr raptor_brake_pub_;
        rclcpp::Publisher<SteeringCmd>::SharedPtr raptor_steer_pub_;

        rclcpp::Subscription<VehicleControl>::SharedPtr tai_cmd_sub_;
        rclcpp::Subscription<AcceleratorPedalCmd>::SharedPtr raptor_throttle_sub_;
        rclcpp::Subscription<BrakeCmd>::SharedPtr raptor_brake_sub_;
        rclcpp::Subscription<SteeringCmd>::SharedPtr raptor_steer_sub_;

        rclcpp::TimerBase::SharedPtr tai_pub_timer_;
        rclcpp::TimerBase::SharedPtr tai_pub_watchdog_timer_;

        AcceleratorPedalCmd::SharedPtr raptor_throttle;
        BrakeCmd::SharedPtr raptor_brake;
        SteeringCmd::SharedPtr raptor_steer;

        std::mutex raptor_mutex;

        bool tai_watchdog_triggered[3] = {false, false, false};

        void tai_callback(const VehicleControl::SharedPtr tai_cmd);
        void raptor_throttle_callback(const AcceleratorPedalCmd::SharedPtr raptor_throttle);
        void raptor_steer_callback(const SteeringCmd::SharedPtr raptor_steer);
        void raptor_brake_callback(const BrakeCmd::SharedPtr raptor_brake);
        void raptor_to_tai_timer_callback();
        void raptor_to_tai_watchdog_timer_callback();
        void reset_raptor_controls();

        template <typename T>
        void reset_raptor_controls(std::shared_ptr<T> & t);
    };
}

#endif //raptor_tai_interface_HPP_
