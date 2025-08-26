#pragma once


#include "base_tf.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
namespace jszr_tf
{
    class MujocoTF : public BaseTF
    {
    public:
        MujocoTF(const rclcpp::Node::WeakPtr& node);
        ~MujocoTF() = default;

        void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) override;
        bool InitParams() override;
        bool InitMember() override;

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_ptr_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    pose_pub_ptr_;
        std::shared_ptr<tf2_ros::TransformBroadcaster>           tf_broad_caster_ptr_;

        geometry_msgs::msg::TransformStamped base_2_odom_tf_;
        geometry_msgs::msg::TransformStamped odom_2_map_tf_;
        std::string                          input_mujoco_pose_topic_;
        std::string                          output_pose_topic_;
    };

}  // namespace jszr_tf