/**
 * @file transform_manager_node.hpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Publish all transforms
 * @version 0.1
 * @date 2021-11-28
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */

#ifndef TRANSFORM_MANAGER__TRANSFORM_MANAGER_NODE_HPP_
#define TRANSFORM_MANAGER__TRANSFORM_MANAGER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace tritonai
{
    namespace localization
    {
        using geometry_msgs::msg::PoseWithCovarianceStamped;
        using nav_msgs::msg::Odometry;
        using std::placeholders::_1;

        class TransformManagerNode : public rclcpp::Node
        {
        public:
            explicit TransformManagerNode(const rclcpp::NodeOptions &options);

        private:
            std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
            rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr map_pose_sub_;

            void odom_callback_(const Odometry::SharedPtr odom_msg);
            void map_pose_callback_(const PoseWithCovarianceStamped::SharedPtr pose_msg);
        };

    }
}

#endif // TRANSFORM_MANAGER__TRANSFORM_MANAGER_NODE_HPP_