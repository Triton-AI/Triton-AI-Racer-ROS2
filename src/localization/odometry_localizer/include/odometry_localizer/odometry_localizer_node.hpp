/**
 * @file odometry_localizer_node.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Localize with odometry and IMU
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */

#ifndef ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_NODE_HPP_
#define ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "odometry_localizer/odometry_localizer.hpp"

namespace tritonai
{
    namespace localization
    {
        using nav_msgs::msg::Odometry;
        using sensor_msgs::msg::Imu;
        using std::placeholders::_1;

        class OdometryLocalizerNode : public rclcpp::Node
        {
        public:
            explicit OdometryLocalizerNode(const rclcpp::NodeOptions &options);

        private:
            rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
            rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
            OdometryLocalizer localizer_;
            bool use_magnetometer_;
            void imu_callback(const Imu::SharedPtr imu_msg);
        };

    }
}

#endif // ODOMETRY_LOCALIZER__ODOMETRY_LOCALIZER_NODE_HPP_