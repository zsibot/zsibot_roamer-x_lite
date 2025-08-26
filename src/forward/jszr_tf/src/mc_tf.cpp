#include "mc_tf.h"

namespace jszr_tf
{
    MCTF::MCTF(const rclcpp::Node::WeakPtr& node)
        : BaseTF(node)
    {
        if (!InitParams())
        {
            RCLCPP_INFO(node_.lock()->get_logger(), "Params and Member Init Failed!!!!");
        }
    }

    bool MCTF::InitParams()
    {
        return InitMember();
    }

    bool MCTF::InitMember()
    {
        map_odom_tf_.header.frame_id = map_frame_;
        map_odom_tf_.child_frame_id  = odom_frame_;

        map_odom_tf_.transform.translation.x = 0.0;
        map_odom_tf_.transform.translation.y = 0.0;
        map_odom_tf_.transform.translation.z = 0.0;

        map_odom_tf_.transform.rotation.x = 0.0;
        map_odom_tf_.transform.rotation.y = 0.0;
        map_odom_tf_.transform.rotation.z = 0.0;
        map_odom_tf_.transform.rotation.w = 1.0;

        odom_base_tf_.header.frame_id = odom_frame_;
        odom_base_tf_.child_frame_id  = base_frame_;

        tf_broadcaster_ptr_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_.lock());
        return true;
    }

    void MCTF::PublishTF(const nav_msgs::msg::Odometry& msg)
    {
        map_odom_tf_.header.stamp             = msg.header.stamp;
        odom_base_tf_.header.stamp            = msg.header.stamp;
        odom_base_tf_.transform.translation.x = msg.pose.pose.position.x;
        odom_base_tf_.transform.translation.y = msg.pose.pose.position.y;
        odom_base_tf_.transform.translation.z = msg.pose.pose.position.z;
        odom_base_tf_.transform.rotation      = msg.pose.pose.orientation;
        tf_broadcaster_ptr_->sendTransform(odom_base_tf_);
        tf_broadcaster_ptr_->sendTransform(map_odom_tf_);
    }

}  // namespace jszr_tf
