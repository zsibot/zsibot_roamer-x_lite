/**
 * @file localization_tf.cpp
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-03-19
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#include "localization_tf.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
namespace jszr_tf
{
    LocalizationTF::LocalizationTF(const rclcpp::Node::WeakPtr& node)
        : BaseTF(node)
    {
        if (!InitParams())
        {
            RCLCPP_INFO(node_.lock()->get_logger(), "Params and Member Init Failed!!!!");
        }

        subscribeOdometry(uniform_odom_topic_);

        record_path_service_ = node_.lock()->create_service<robots_dog_msgs::srv::RecordPath>(
            "record_path", std::bind(&LocalizationTF::record_path_service_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool LocalizationTF::InitMember()
    {
        odom_2_map_tf_.header.frame_id         = map_frame_;
        odom_2_map_tf_.child_frame_id          = odom_frame_;
        odom_2_map_tf_.transform.translation.x = 0.0;
        odom_2_map_tf_.transform.translation.y = 0.0;
        odom_2_map_tf_.transform.translation.z = 0.0;
        odom_2_map_tf_.transform.rotation.x    = 0.0;
        odom_2_map_tf_.transform.rotation.y    = 0.0;
        odom_2_map_tf_.transform.rotation.z    = 0.0;
        odom_2_map_tf_.transform.rotation.w    = 1.0;

        base_2_odom_tf_.header.frame_id = odom_frame_;
        base_2_odom_tf_.child_frame_id  = base_frame_;
        tf_broad_caster_ptr_            = std::make_shared<tf2_ros::TransformBroadcaster>(node_.lock());
        return true;
    }

    bool LocalizationTF::InitParams()
    {
        node_.lock()->get_parameter("uniform_odom_topic", uniform_odom_topic_);

        return InitMember();
    }

    void LocalizationTF::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        base_2_odom_tf_.header.stamp = msg->header.stamp;

        base_2_odom_tf_.transform.translation.x = msg->pose.pose.position.x;
        base_2_odom_tf_.transform.translation.y = msg->pose.pose.position.y;
        base_2_odom_tf_.transform.translation.z = 0.05;
        tf2::Quaternion tf2_quat(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Matrix3x3 rotation(tf2_quat);
        double         roll, pitch, yaw;
        rotation.getRPY(roll, pitch, yaw);
        tf2_quat.setRPY(0.0, 0.0, yaw);
        base_2_odom_tf_.transform.rotation.x = tf2_quat.x();
        base_2_odom_tf_.transform.rotation.y = tf2_quat.y();
        base_2_odom_tf_.transform.rotation.z = tf2_quat.z();
        base_2_odom_tf_.transform.rotation.w = tf2_quat.w();
        tf_broad_caster_ptr_->sendTransform(base_2_odom_tf_);

        odom_2_map_tf_.header.stamp = msg->header.stamp;
        tf_broad_caster_ptr_->sendTransform(odom_2_map_tf_);

        // path record
        if (record_)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp    = msg->header.stamp;
            pose.header.frame_id = "map";
            pose.pose            = msg->pose.pose;
            recorded_path_->poses.push_back(pose);

            if (add_key_pose_)
            {
                key_poses_->push_back(pose);

                add_key_pose_ = false;
            }
        }
    }

    void LocalizationTF::record_path_service_callback(const std::shared_ptr<robots_dog_msgs::srv::RecordPath::Request> request,
        std::shared_ptr<robots_dog_msgs::srv::RecordPath::Response>                                                    response)
    {
        response->success = true;

        if (request->cmd == robots_dog_msgs::srv::RecordPath::Request::CMD_START)
        {
            if (record_)
            {
                response->success = false;
                response->message = "Already in recording mode.";
            }
            else
            {
                recorded_path_                  = std::make_shared<nav_msgs::msg::Path>();
                recorded_path_->header.frame_id = "map";
                recorded_path_->header.stamp    = node_.lock()->now();

                key_poses_ = std::make_shared<std::vector<geometry_msgs::msg::PoseStamped>>();

                record_ = true;
            }
        }
        else if (request->cmd == robots_dog_msgs::srv::RecordPath::Request::CMD_ADD_KEY_POSE)
        {
            if (record_)
            {
                add_key_pose_ = true;
            }
            else
            {
                response->success = false;
                response->message = "Not in recording mode.";
            }
        }
        else if (request->cmd == robots_dog_msgs::srv::RecordPath::Request::CMD_STOP_AND_SAVE)
        {
            if (record_)
            {
                record_ = false;

                // TODO save
                recorded_path_.reset();
                key_poses_.reset();
            }
            else
            {
                response->success = false;
                response->message = "Not in recording mode.";
            }
        }
        else if (request->cmd == robots_dog_msgs::srv::RecordPath::Request::CMD_STOP_AND_DISCARD)
        {
            if (record_)
            {
                record_ = false;

                recorded_path_.reset();
                key_poses_.reset();
            }
            else
            {
                response->success = false;
                response->message = "Not in recording mode.";
            }
        }
    }

}  // namespace jszr_tf
