#include "gazebo_tf.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace jszr_tf
{
    GazeboTF::GazeboTF(const rclcpp::Node::WeakPtr& node)
        : BaseTF(node)
    {
        if (!InitParams())
        {
            RCLCPP_INFO(node_.lock()->get_logger(), "Params and Member Init Failed!!!!");
        }

        pose_pub_ptr_ = node_.lock()->create_publisher<nav_msgs::msg::Odometry>(output_pose_topic_, 10);

        subscribeOdometry(input_gazebo_pose_topic_);
    }

    bool GazeboTF::InitParams()
    {
        node_.lock()->declare_parameter<std::string>("input_gazebo_pose_topic", "/odom/gazebo");
        node_.lock()->declare_parameter<std::string>("output_pose_topic", "/odom/ground_truth");

        node_.lock()->get_parameter("input_gazebo_pose_topic", input_gazebo_pose_topic_);
        node_.lock()->get_parameter("output_pose_topic", output_pose_topic_);

        return InitMember();
    }

    bool GazeboTF::InitMember()
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

        RCLCPP_INFO(node_.lock()->get_logger(), "gazebo odom started");
        return true;
    }

    void GazeboTF::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose_pub_ptr_->publish(*msg);

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
    }

}  // namespace jszr_tf
