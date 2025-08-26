#include "base_tf.h"

#include <chrono>
#include <cmath>
#include <common.h>
#include <cstdlib>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

ZSIBOT_IMPL_TOPIC(NavCmd, Ecal);

ZSIBOT_IMPL_TOPIC(NavState, Ecal);

namespace jszr_tf
{
    BaseTF::BaseTF(const rclcpp::Node::WeakPtr& node)
        : node_(node)
    {
        if (!InitParams())
        {
            RCLCPP_INFO(node_.lock()->get_logger(), "Params and Member Init Failed!!!!");
        }
    }

    BaseTF::~BaseTF() {}

    bool BaseTF::InitParams()
    {
        node_.lock()->declare_parameter("platform", "NX_XG3588");
        node_.lock()->declare_parameter("mc_odom_topic", "/odom/mc_odom");
        node_.lock()->declare_parameter("uniform_odom_topic", "/odom/ground_truth");
        node_.lock()->declare_parameter("joint_states_topic", "/joint_states/mc");
        node_.lock()->declare_parameter("joint_states_cmd_topic", "/joint_states/gui_cmd");
        node_.lock()->declare_parameter("low_level_cmd_topic", "/motor_cmd");
        node_.lock()->declare_parameter("mc_controller_type", "");
        node_.lock()->declare_parameter("filter_nums", 10);
        node_.lock()->declare_parameter("sleep_ms", 2);

        node_.lock()->declare_parameter<std::string>("map_frame", "map");
        node_.lock()->declare_parameter<std::string>("odom_frame", "odom");
        node_.lock()->declare_parameter<std::string>("base_frame", "base_link");

        node_.lock()->get_parameter("map_frame", map_frame_);
        node_.lock()->get_parameter("odom_frame", odom_frame_);
        node_.lock()->get_parameter("base_frame", base_frame_);

        node_.lock()->get_parameter("tf_type", tf_type_);
        node_.lock()->get_parameter("platform", platform_);
        node_.lock()->get_parameter("mc_odom_topic", mc_odom_topic_);
        node_.lock()->get_parameter("uniform_odom_topic", uniform_odom_topic_);
        node_.lock()->get_parameter("joint_states_topic", joint_states_topic_);
        node_.lock()->get_parameter("joint_states_cmd_topic", joint_states_cmd_topic_);
        node_.lock()->get_parameter("low_level_cmd_topic", low_level_cmd_topic_);
        node_.lock()->get_parameter("sleep_ms", sleep_ms_);
        node_.lock()->get_parameter("mc_controller_type", mc_controller_type_);
        RCLCPP_INFO(node_.lock()->get_logger(), "tf_type : %s", tf_type_.c_str());

        return InitMember();
    }

    bool BaseTF::InitMember()
    {
        if (tf_type_ == "localization_tf" || tf_type_ == "mc_tf")
        {
            if (tf_type_ == "mc_tf")
            {
                RCLCPP_INFO(node_.lock()->get_logger(), "enter mc tf");
                odom_pub_ptr_ = node_.lock()->create_publisher<nav_msgs::msg::Odometry>(uniform_odom_topic_, 1);
            }
            else if (tf_type_ == "localization_tf")
            {
                RCLCPP_INFO(node_.lock()->get_logger(), "enter localization tf");
                odom_pub_ptr_ = node_.lock()->create_publisher<nav_msgs::msg::Odometry>(mc_odom_topic_, 10);
            }
            else
            {
                RCLCPP_ERROR(node_.lock()->get_logger(), "\033[1;31mError: Unknown TF_TYPE\033[0m");
            }

            RCLCPP_INFO(node_.lock()->get_logger(), "init mc joint states pub");
            joint_states_pub_ptr_ = node_.lock()->create_publisher<sensor_msgs::msg::JointState>(joint_states_topic_, 10);

            RCLCPP_INFO(node_.lock()->get_logger(), "init mc low level cmd pub");
            low_level_cmd_pub_ptr_ = node_.lock()->create_publisher<mc_sdk_msg::msg::LowLevelCmd>(low_level_cmd_topic_, 10);
            RCLCPP_INFO(node_.lock()->get_logger(), "init joint states cmd sub");
            joint_states_cmd_sub_ptr_ = node_.lock()->create_subscription<sensor_msgs::msg::JointState>(
                joint_states_cmd_topic_, 10, std::bind(&BaseTF::HandleJointStatesCallback, this, std::placeholders::_1));

            InitConnectWithMc();
        }

        if (tf_type_ != "gazebo_tf")
        {
            if (mc_controller_type_ == "RL_TRACK_PATH")
            {
                planner_vel_with_mc_traj_cmd_subscriber_ = node_.lock()->create_subscription<robots_dog_msgs::msg::CmdVelWithTrajectory>(
                    "/cmd_vel_with_mc_trajectory", rclcpp::SystemDefaultsQoS(),
                    std::bind(&BaseTF::HandlePlannerVelWithMcTrajectoryCallback, this, std::placeholders::_1));
            }
            else
            {
                planner_vel_cmd_subscriber_ = node_.lock()->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", 10, std::bind(&BaseTF::HandlePlannerVelCallback, this, std::placeholders::_1));
            }
        }

        if (tf_type_ == "mujoco_tf")
        {
            ZSIBOT_UNUSED(pub_.init("nav_cmd"));
        }

        return true;
    }

    void BaseTF::InitConnectWithMc()
    {
        RCLCPP_INFO(node_.lock()->get_logger(), "enter mc InitUDP");
        ZSIBOT_UNUSED(sub_.init("nav_state"));
        ZSIBOT_UNUSED(pub_.init("nav_cmd"));
        sub_.subscribe(
            [this](const zsibot::rmw::MessagePtr<robot_sdk::pb::NavigationState>& msg)
            {
                PublishOdometry(msg->data);
                PublishJointStates(msg->data);
            });
        // mc_control_thread_ = std::thread(&BaseTF::RequestMcCallback, this);
    }

    void BaseTF::RequestMcCallback()
    {
        RCLCPP_ERROR(node_.lock()->get_logger(), "enter mc UDPProcessCallBack");

        while (rclcpp::ok())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms_));
        }
    }

    void BaseTF::PublishOdometry(const robot_sdk::pb::NavigationState& state_msg)
    {
        geometry_msgs::msg::Quaternion odom_quat;
        odom_quat.w = state_msg.quat()[0];
        odom_quat.x = state_msg.quat()[1];
        odom_quat.y = state_msg.quat()[2];
        odom_quat.z = state_msg.quat()[3];

        auto odom_msg            = nav_msgs::msg::Odometry();
        odom_msg.header.stamp    = node_.lock()->now();
        odom_msg.header.frame_id = odom_frame_;

        odom_msg.pose.pose.position.x  = state_msg.position()[0];
        odom_msg.pose.pose.position.y  = state_msg.position()[1];
        odom_msg.pose.pose.position.z  = state_msg.position()[2];
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.child_frame_id        = base_frame_;
        odom_msg.twist.twist.linear.x  = state_msg.v_world()[0];
        odom_msg.twist.twist.linear.y  = state_msg.v_world()[1];
        odom_msg.twist.twist.angular.z = state_msg.omega_world()[2];

        odom_pub_ptr_->publish(odom_msg);

        if (tf_type_ == "mc_tf")
        {
            PublishTF(odom_msg);
        }
    }

    void BaseTF::PublishJointStates(const robot_sdk::pb::NavigationState& state_msg)
    {
        sensor_msgs::msg::JointState js_msg;
        js_msg.header.stamp    = node_.lock()->now();
        js_msg.header.frame_id = base_frame_;

        js_msg.name = { "FR_ABAD_JOINT", "FR_HIP_JOINT", "FR_KNEE_JOINT", "FL_ABAD_JOINT", "FL_HIP_JOINT", "FL_KNEE_JOINT", "RR_ABAD_JOINT",
            "RR_HIP_JOINT", "RR_KNEE_JOINT", "RL_ABAD_JOINT", "RL_HIP_JOINT", "RL_KNEE_JOINT" };

        js_msg.position = { state_msg.q_abad()[0], state_msg.q_hip()[0], state_msg.q_knee()[0], state_msg.q_abad()[1], state_msg.q_hip()[1],
            state_msg.q_knee()[1], state_msg.q_abad()[2], state_msg.q_hip()[2], state_msg.q_knee()[2], state_msg.q_abad()[3],
            state_msg.q_hip()[3], state_msg.q_knee()[3] };

        js_msg.velocity = { state_msg.qd_abad()[0], state_msg.qd_hip()[0], state_msg.qd_knee()[0], state_msg.qd_abad()[1],
            state_msg.qd_hip()[1], state_msg.qd_knee()[1], state_msg.qd_abad()[2], state_msg.qd_hip()[2], state_msg.qd_knee()[2],
            state_msg.qd_abad()[3], state_msg.qd_hip()[3], state_msg.qd_knee()[3] };

        if (!joint_states_pub_ptr_)
        {
            RCLCPP_ERROR(node_.lock()->get_logger(), "joint_states_pub_ptr_ is null, cannot publish JointState");
            return;
        }
        joint_states_pub_ptr_->publish(js_msg);
    }

    void BaseTF::PublishTF(const nav_msgs::msg::Odometry& msg)
    {
        RCLCPP_WARN(node_.lock()->get_logger(), "BaseTF::OdometryCallback called, but not overridden!");
    }

    void BaseTF::subscribeOdometry(const std::string& topic_name)
    {
        pose_sub_ptr_ = node_.lock()->create_subscription<nav_msgs::msg::Odometry>(
            topic_name, 10, std::bind(&BaseTF::OdometryCallback, this, std::placeholders::_1));
    }

    void BaseTF::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_WARN(node_.lock()->get_logger(), "BaseTF::OdometryCallback called, but not overridden!");
    }

    void BaseTF::HandlePlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_WARN(node_.lock()->get_logger(), "OK+++++");
        pb_cmd_.Clear();
        pb_cmd_.set_vx(std::fabs(msg->linear.x) < 0.085 ? 0.0 : msg->linear.x);
        pb_cmd_.set_vy(std::fabs(msg->linear.y) < 0.085 ? 0.0 : msg->linear.y);
        pb_cmd_.set_yaw_rate(msg->angular.z);
        pub_.publish(pb_cmd_);
    }

    void BaseTF::HandlePlannerVelWithMcTrajectoryCallback(const robots_dog_msgs::msg::CmdVelWithTrajectory::SharedPtr msg)
    {

        pb_cmd_.Clear();
        pb_cmd_.set_vx(std::fabs(msg->cmd_vel.twist.linear.x) < 0.085 ? 0.0 : msg->cmd_vel.twist.linear.x);
        pb_cmd_.set_vy(std::fabs(msg->cmd_vel.twist.linear.y) < 0.085 ? 0.0 : msg->cmd_vel.twist.linear.y);
        pb_cmd_.set_yaw_rate(msg->cmd_vel.twist.angular.z);

        if (msg->trajectory.poses.size() != 20)
        {
            RCLCPP_ERROR(node_.lock()->get_logger(), "==> Received  traj size is not equal to mc reuqired trajectory size");
            RCLCPP_ERROR(node_.lock()->get_logger(), "Received traj size : %ld | Mc trajectory : %d", msg->trajectory.poses.size(), 20);
            return;
        }

        for (size_t i = 0; i < msg->trajectory.poses.size(); ++i)
        {
            pb_cmd_.add_trajectory();
            pb_cmd_.mutable_trajectory()->mutable_data()[i]->set_x(msg->trajectory.poses[i].pose.position.x);
            pb_cmd_.mutable_trajectory()->mutable_data()[i]->set_y(msg->trajectory.poses[i].pose.position.y);


            tf2::Quaternion q(msg->trajectory.poses[i].pose.orientation.x, msg->trajectory.poses[i].pose.orientation.y,
                msg->trajectory.poses[i].pose.orientation.z, msg->trajectory.poses[i].pose.orientation.w);
            double          roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            pb_cmd_.mutable_trajectory()->mutable_data()[i]->set_theta(yaw);
        }
        pub_.publish(pb_cmd_);
        RCLCPP_INFO(node_.lock()->get_logger(), "==> 发送了: %f %f %f", pb_cmd_.vx(), pb_cmd_.vy(), pb_cmd_.yaw_rate());
    }

    void BaseTF::HandleJointStatesCallback(const sensor_msgs::msg::JointState& state_msg)
    {
        if (state_msg.name.size() != state_msg.position.size())
        {
            RCLCPP_ERROR(node_.lock()->get_logger(), "JointState invalid: name.size()=%zu != position.size()=%zu", state_msg.name.size(),
                state_msg.position.size());
            return;
        }

        static const std::array<std::string, 12> kOrder = { "FR_ABAD_JOINT", "FR_HIP_JOINT", "FR_KNEE_JOINT", "FL_ABAD_JOINT",
            "FL_HIP_JOINT", "FL_KNEE_JOINT", "RR_ABAD_JOINT", "RR_HIP_JOINT", "RR_KNEE_JOINT", "RL_ABAD_JOINT", "RL_HIP_JOINT",
            "RL_KNEE_JOINT" };

        std::unordered_map<std::string, double> pos_map;
        pos_map.reserve(state_msg.name.size());

        std::unordered_set<std::string> seen;
        for (size_t i = 0; i < state_msg.name.size(); ++i)
        {
            const auto& name = state_msg.name[i];
            if (name.empty())
            {
                RCLCPP_WARN(node_.lock()->get_logger(), "JointState contains empty joint name at index %zu, ignored", i);
                continue;
            }

            const double q = state_msg.position[i];
            if (!std::isfinite(q))
            {
                RCLCPP_ERROR(node_.lock()->get_logger(), "Joint '%s' has non-finite position (NaN/Inf), abort publish", name.c_str());
                return;
            }

            if (seen.count(name))
            {
                RCLCPP_WARN(node_.lock()->get_logger(), "Duplicate joint name '%s' in JointState, last value will be used", name.c_str());
            }
            seen.insert(name);
            pos_map[name] = q;
        }

        mc_sdk_msg::msg::LowLevelCmd cmd_msg;

        for (size_t i = 0; i < kOrder.size(); ++i)
        {
            const auto& jn = kOrder[i];
            const auto  it = pos_map.find(jn);
            if (it == pos_map.end())
            {
                RCLCPP_ERROR(node_.lock()->get_logger(), "Missing joint '%s' in JointState (required to fill q_des[%zu])", jn.c_str(), i);
                return;
            }
            cmd_msg.q_des[i] = static_cast<float>(it->second);
        }

        // TODO: qd_des, kp, kd

        if (!low_level_cmd_pub_ptr_)
        {
            RCLCPP_ERROR(node_.lock()->get_logger(), "low_level_cmd_pub_ptr_ is null, cannot publish LowLevelCmd");
            return;
        }
        low_level_cmd_pub_ptr_->publish(cmd_msg);
    }

}  // namespace jszr_tf
