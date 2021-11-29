/**
 * @file transform_manager_node.cpp
 * @author Haoru Xue (hxue@ucsd.edu)
 * @brief Localize with odometry and IMU
 * @version 0.1
 * @date 2021-11-27
 * 
 * @copyright Copyright (c) 2021 [Triton AI]
 * 
 */
#include <functional>
#include <string>
#include <rclcpp/qos.hpp>
#include "transform_manager/transform_manager_node.hpp"

namespace tritonai
{
  namespace localization
  {
    using geometry_msgs::msg::TransformStamped;

    TransformManagerNode::TransformManagerNode(const rclcpp::NodeOptions &options)
        : rclcpp::Node("pilot_node", options)
    {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      std::string odom_topic = declare_parameter("odom_topic", "odom/localizer");
      odom_sub_ = create_subscription<Odometry>(odom_topic, rclcpp::QoS{10}, std::bind(&TransformManagerNode::odom_callback_, this, _1));
      std::string pose_topic = declare_parameter("map_pose_topic", "pose");
      map_pose_sub_ = create_subscription<PoseWithCovarianceStamped>(pose_topic, rclcpp::QoS{10}, std::bind(&TransformManagerNode::map_pose_callback_, this, _1));
    }
    void TransformManagerNode::odom_callback_(const Odometry::SharedPtr odom_msg)
    {
      TransformStamped t;
      t.header.stamp = odom_msg->header.stamp;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";
      t.transform.translation.x = odom_msg->pose.pose.position.x;
      t.transform.translation.y = odom_msg->pose.pose.position.y;
      t.transform.translation.z = odom_msg->pose.pose.position.z;
      t.transform.rotation = odom_msg->pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);
    }
    void TransformManagerNode::map_pose_callback_(const PoseWithCovarianceStamped::SharedPtr pose_msg)
    {
      TransformStamped odom_to_bl_transform;
      try
      {
        odom_to_bl_transform = tf_buffer_->lookupTransform("base_link", "odom", rclcpp::Time(pose_msg->header.stamp));
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            "odom", "base_link", ex.what());
        return;
      }
      auto &pose = pose_msg->pose.pose;
      TransformStamped map_to_odom_transform;
      map_to_odom_transform.header.frame_id = "map";
      map_to_odom_transform.child_frame_id = "odom";
      map_to_odom_transform.header.stamp = pose_msg->header.stamp;
      map_to_odom_transform.transform.translation.x = pose.position.x - odom_to_bl_transform.transform.translation.x;
      map_to_odom_transform.transform.translation.y = pose.position.y - odom_to_bl_transform.transform.translation.y;
      map_to_odom_transform.transform.translation.z = pose.position.z - odom_to_bl_transform.transform.translation.z;
      auto q1 = tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
      auto q2 = tf2::Quaternion(odom_to_bl_transform.transform.rotation.x, odom_to_bl_transform.transform.rotation.y, odom_to_bl_transform.transform.rotation.z, odom_to_bl_transform.transform.rotation.w);
      auto q = q1 - q2;
      map_to_odom_transform.transform.rotation.x = q.x();
      map_to_odom_transform.transform.rotation.y = q.y();
      map_to_odom_transform.transform.rotation.z = q.z();
      map_to_odom_transform.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(map_to_odom_transform);
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tritonai::localization::TransformManagerNode)