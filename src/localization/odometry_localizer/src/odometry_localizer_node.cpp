/**
 * @file odometry_localizer_node.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Localize with odometry and IMU
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */
#include <functional>
#include <rclcpp/qos.hpp>
#include "odometry_localizer/odometry_localizer_node.hpp"

namespace tritonai
{
  namespace localization
  {

    OdometryLocalizerNode::OdometryLocalizerNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("pilot_node", options)
    {
      localizer_ = OdometryLocalizer();
      use_magnetometer_ = declare_parameter("use_magnetometer", false);
      odom_pub_ = create_publisher<Odometry>("odom/localizer", rclcpp::QoS{10});
      imu_sub_ = create_subscription<Imu>("imu/raw", rclcpp::SensorDataQoS(), std::bind(&OdometryLocalizerNode::imu_callback, this, _1));
    }

    void OdometryLocalizerNode::imu_callback(const Imu::SharedPtr imu_msg)
    {
      int64_t timestamp = rclcpp::Time(imu_msg->header.stamp).nanoseconds();
      localizer_.update_lin_acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z, timestamp);
      if (use_magnetometer_)
      {
        // RCLCPP_INFO(get_logger(), "%f, %f, %f, %f", imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w );
        localizer_.update_orientation(imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);
      }
      else
      {
        localizer_.update_ang_vel(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z, timestamp);
      }

      auto position = localizer_.get_position();
      auto orientation = localizer_.get_orientation();
      auto velocity = localizer_.get_velocity();

      Odometry odom_msg = Odometry();
      odom_msg.header.frame_id = "odom";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = imu_msg->header.stamp;
      odom_msg.pose.pose.position.x = position.x();
      odom_msg.pose.pose.position.y = position.y();
      odom_msg.pose.pose.position.z = position.z();
      odom_msg.pose.pose.orientation.x = orientation.x();
      odom_msg.pose.pose.orientation.y = orientation.y();
      odom_msg.pose.pose.orientation.z = orientation.z();
      odom_msg.pose.pose.orientation.w = orientation.w();
      odom_msg.twist.twist.angular = imu_msg->angular_velocity;
      odom_msg.twist.twist.linear.x = velocity.x();
      odom_msg.twist.twist.linear.y = velocity.y();
      odom_msg.twist.twist.linear.z = velocity.z();

      odom_pub_->publish(odom_msg);
    }

  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tritonai::localization::OdometryLocalizerNode)