/**
 * @file pub_tf.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-03-11
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */
#pragma once

#include "base_tf.h"
#include "robots_dog_msgs/srv/record_path.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
namespace jszr_tf
{
    class LocalizationTF : public BaseTF
    {
    public:
        explicit LocalizationTF(const rclcpp::Node::WeakPtr& node);

        bool InitParams() override;
        bool InitMember() override;

    private:
        void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) override;

        /**
         * @brief The service callback to react to client's request
         * @param request to the service
         * @param response from the service
         */
        void record_path_service_callback(const std::shared_ptr<robots_dog_msgs::srv::RecordPath::Request> request,
            std::shared_ptr<robots_dog_msgs::srv::RecordPath::Response>                                    response);

        std::shared_ptr<tf2_ros::TransformBroadcaster>                   tf_broad_caster_ptr_;
        rclcpp::Service<robots_dog_msgs::srv::RecordPath>::SharedPtr record_path_service_;

        bool                                                          record_{ false };
        bool                                                          add_key_pose_{ false };
        std::shared_ptr<nav_msgs::msg::Path>                          recorded_path_;
        std::shared_ptr<std::vector<geometry_msgs::msg::PoseStamped>> key_poses_;

        geometry_msgs::msg::TransformStamped base_2_odom_tf_;
        geometry_msgs::msg::TransformStamped odom_2_map_tf_;
        std::string                          uniform_odom_topic_;
    };

}  // namespace jszr_tf