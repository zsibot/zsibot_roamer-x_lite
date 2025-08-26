/**
 * @file mc_tf.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-03-19
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */
#pragma once

#include "base_tf.h"
namespace jszr_tf
{
    class MCTF : public BaseTF
    {
    public:
        explicit MCTF(const rclcpp::Node::WeakPtr& node);

        ~MCTF() = default;

    private:
        bool InitMember();

        bool InitParams();

        void PublishTF(const nav_msgs::msg::Odometry& msg);

    private:
        geometry_msgs::msg::TransformStamped map_odom_tf_;
        geometry_msgs::msg::TransformStamped odom_base_tf_;

        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ptr_;
    };
}  // namespace jszr_tf