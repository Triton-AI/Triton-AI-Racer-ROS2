#include "raptor_tai_interfaces/raptor_tai_interfaces.hpp"
#include <chrono>

namespace raptor_tai
{

    using namespace std::chrono_literals;
    using std::placeholders::_1;

    RaptorTaiInterface::RaptorTaiInterface(const rclcpp::NodeOptions &options)
        : rclcpp::Node("raptor_tai_node", options)
    {
        reset_raptor_controls();
        if (declare_parameter<bool>("tai_cmd_to_raptor_cmd", false))
        // Triton AI Control => Raptor Control (Pilot => Raptor CAN)
        {
            tai_cmd_sub_ = create_subscription<TritonAIRacerControl>(
                "vehicle_cmd_to_raptor", 1, std::bind(&RaptorTaiInterface::tai_callback, this, _1));
            raptor_brake_pub_ = create_publisher<BrakeCmd>(
                "brake_cmd_to_raptor", 1);
            raptor_throttle_pub_ = create_publisher<AcceleratorPedalCmd>(
                "accelerator_pedal_cmd_to_raptor", 1);
            raptor_steer_pub_ = create_publisher<SteeringCmd>(
                "steering_cmd_to_raptor", 1);
        }

        if (declare_parameter<bool>("raptor_cmd_to_tai_cmd", false))
        // Raptor Control => Triton AI Control (Raptor JS => Pilot)
        {
            tai_cmd_pub_ = create_publisher<TritonAIRacerControl>(
                "vehicle_cmd_from_raptor", 1);
            raptor_brake_sub_ = create_subscription<BrakeCmd>(
                "brake_cmd_from_raptor", 1, std::bind(&RaptorTaiInterface::raptor_brake_callback, this, _1));
            raptor_throttle_sub_ = create_subscription<AcceleratorPedalCmd>(
                "accelerator_pedal_cmd_from_raptor", 1, std::bind(&RaptorTaiInterface::raptor_throttle_callback, this, _1));
            raptor_steer_sub_ = create_subscription<SteeringCmd>(
                "steering_cmd_from_raptor", 1, std::bind(&RaptorTaiInterface::raptor_steer_callback, this, _1));

            int tai_cmd_freq = declare_parameter<int>("tai_cmd_freq", 20);
            tai_pub_timer_ = create_wall_timer(std::chrono::microseconds(1000000 / tai_cmd_freq), std::bind(&RaptorTaiInterface::raptor_to_tai_timer_callback, this));

            float tai_cmd_timeout_s = declare_parameter<float>("tai_cmd_timeout_s", 0.2);
            tai_pub_watchdog_timer_ = create_wall_timer(std::chrono::microseconds((int) (1000000 * tai_cmd_timeout_s)), std::bind(&RaptorTaiInterface::raptor_to_tai_watchdog_timer_callback, this));
        }
    }

    void RaptorTaiInterface::tai_callback(const TritonAIRacerControl::SharedPtr tai_cmd)
    {
        auto throttle_msg = AcceleratorPedalCmd();
        throttle_msg.pedal_cmd = tai_cmd->throttle.throttle;
        throttle_msg.enable = true;
        raptor_throttle_pub_->publish(throttle_msg);

        auto steer_msg = SteeringCmd();
        steer_msg.angle_cmd = tai_cmd->steering_rad.steer;
        steer_msg.enable = true;
        raptor_steer_pub_->publish(steer_msg);

        auto brake_msg = BrakeCmd();
        brake_msg.pedal_cmd = tai_cmd->brake.brake;
        steer_msg.enable = true;
        raptor_brake_pub_->publish(brake_msg);
    }

    void RaptorTaiInterface::raptor_throttle_callback(const AcceleratorPedalCmd::SharedPtr raptor_throttle)
    {
        std::lock_guard<std::mutex> guard(raptor_mutex);
        this->raptor_throttle = raptor_throttle;
        tai_watchdog_triggered[0] = false;
    }

    void RaptorTaiInterface::raptor_steer_callback(const SteeringCmd::SharedPtr raptor_steer)
    {
        std::lock_guard<std::mutex> guard(raptor_mutex);
        this->raptor_steer = raptor_steer;
        tai_watchdog_triggered[1] = false;
    }

    void RaptorTaiInterface::raptor_brake_callback(const BrakeCmd::SharedPtr raptor_brake)
    {
        std::lock_guard<std::mutex> guard(raptor_mutex);
        this->raptor_brake = raptor_brake;
        tai_watchdog_triggered[2] = false;
    }

    void RaptorTaiInterface::raptor_to_tai_timer_callback()
    {
        std::lock_guard<std::mutex> guard(raptor_mutex);
        TritonAIRacerControl tai_cmd = TritonAIRacerControl();
        tai_cmd.throttle.throttle = raptor_throttle->pedal_cmd;
        tai_cmd.steering_rad.steer = raptor_steer->angle_cmd;
        tai_cmd.brake.brake = raptor_brake->pedal_cmd;
        tai_cmd_pub_->publish(tai_cmd);
    }

    template <typename T>
    void RaptorTaiInterface::reset_raptor_controls(std::shared_ptr<T> &t)
    {
        t = std::make_shared<T>();
    }

    void RaptorTaiInterface::reset_raptor_controls()
    {
        reset_raptor_controls<BrakeCmd>(raptor_brake);
        reset_raptor_controls<AcceleratorPedalCmd>(raptor_throttle);
        reset_raptor_controls<SteeringCmd>(raptor_steer);
    }

    void RaptorTaiInterface::raptor_to_tai_watchdog_timer_callback()
    {
        if (tai_watchdog_triggered[0])
        {
            reset_raptor_controls<AcceleratorPedalCmd>(raptor_throttle);
        }
        tai_watchdog_triggered[0] = true;

        if (tai_watchdog_triggered[1])
        {
            reset_raptor_controls<SteeringCmd>(raptor_steer);
        }
        tai_watchdog_triggered[1] = true;

        if (tai_watchdog_triggered[2])
        {
            reset_raptor_controls<BrakeCmd>(raptor_brake);
        }
        tai_watchdog_triggered[2] = true;
    }
}
#include "rclcpp_components/register_node_macro.hpp" // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(raptor_tai::RaptorTaiInterface)