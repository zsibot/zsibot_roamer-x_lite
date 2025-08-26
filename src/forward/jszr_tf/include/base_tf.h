#ifndef BASE_TF_HPP
#define BASE_TF_HPP

#include "highlevel_connector.h"
#include "robots_dog_msgs/msg/cmd_vel_with_trajectory.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mc_sdk_msg/msg/low_level_cmd.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_sdk.pb.h>
#include <sensor_msgs/msg/detail/camera_info__builder.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unordered_map>
#include <zsibot/base.hpp>
#include <zsibot/rmw/ecal.hpp>

ZSIBOT_TOPIC(robot_sdk::pb::NavigationCmd, NavCmd);
ZSIBOT_TOPIC(robot_sdk::pb::NavigationState, NavState);

namespace jszr_tf
{
    class BaseTF
    {
    public:
        BaseTF(const rclcpp::Node::WeakPtr& node);

        virtual ~BaseTF();

        virtual bool InitParams();

        virtual bool InitMember();

    protected:
        /**
         * @brief 初始化与运控的通信
         */
        void InitConnectWithMc();

        void subscribeOdometry(const std::string& topic_name);

        /**
         * @brief 重载 PublishOdometry，基于 pb 数据发送 odom 数据
         * @param [in] state_msg
         */
        void PublishOdometry(const robot_sdk::pb::NavigationState& state_msg);

        /**
         * @brief Publishes joint state information derived from the navigation state message.
         *
         * This function extracts the joint positions, velocities, and names from the provided
         * NavigationState message and populates a JointState message. It then publishes the
         * constructed JointState message for use in downstream components.
         *
         * @param state_msg NavigationState message containing joint data such as positions
         * and velocities.
         */
        void PublishJointStates(const robot_sdk::pb::NavigationState& state_msg);

        /**
         * @brief 与 mc请求线程
         */
        void RequestMcCallback();

        /**
         * @brief 处理 ros2 速度指令，向运控下发指令
         * @param [in] msg
         */
        void HandlePlannerVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        /**
         * @brief 处理 ros2 CmdVelWithTrajectory 指令，向运控下发指令
         * @param msg
         */
        void HandlePlannerVelWithMcTrajectoryCallback(const robots_dog_msgs::msg::CmdVelWithTrajectory::SharedPtr msg);

        /**
         * @brief Handles incoming joint state messages to process joint positions and publishes low-level commands.
         *
         * This callback function validates and processes the received `sensor_msgs::msg::JointState` message. It maps joint
         * names to their respective positions and ensures the consistency of the data. It prepares a low-level command message
         * to update actuator targets, which are then published to the corresponding topic.
         *
         * If the `name` and `position` vector sizes in the joint state message mismatch, or if there are invalid or duplicate
         * joint names, warnings or errors will be logged, and the processing will terminate for that instance.
         *
         * The method adheres to a predefined order of joint names, `kOrder`, to generate the low-level command message.
         *
         * Uses the `low_level_cmd_pub_ptr_` publisher to send the generated low-level command to the appropriate topic.
         *
         * Logs errors if the publisher is unavailable or if any anomalies are detected in the received message.
         *
         * @param state_msg The `sensor_msgs::msg::JointState` message containing joint names and positions.
         */
        void HandleJointStatesCallback(const sensor_msgs::msg::JointState& state_msg);

        virtual void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        virtual void PublishTF(const nav_msgs::msg::Odometry& msg);

    protected:
        rclcpp::Node::WeakPtr                          node_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_ptr_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr    odom_pub_ptr_;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_ptr_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_cmd_sub_ptr_;
        rclcpp::Publisher<mc_sdk_msg::msg::LowLevelCmd>::SharedPtr    low_level_cmd_pub_ptr_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr                  planner_vel_cmd_subscriber_;
        rclcpp::Subscription<robots_dog_msgs::msg::CmdVelWithTrajectory>::SharedPtr planner_vel_with_mc_traj_cmd_subscriber_;

        std::unordered_map<std::string, std::pair<std::string, std::string>> platform_map_{ { "NX_YSC3588",
                                                                                                { "192.168.1.120", "192.168.1.100" } },
            { "NX_XG3588", { "192.168.3.120", "192.168.3.100" } }, { "XG3588", { "127.0.0.1", "127.0.0.1" } } };

        int sleep_ms_;
        int client_port_{ 43909 };
        int service_port_{ 43998 };

        bool last_state_is_good_{ false };

        std::string tf_type_;

        std::string client_ip_;
        std::string service_ip_;
        std::string platform_;
        std::string mc_controller_type_;

        std::string uniform_odom_topic_;
        std::string mc_odom_topic_;

        std::string joint_states_topic_;
        std::string joint_states_cmd_topic_;
        std::string low_level_cmd_topic_;

        std::string pose_topic_;
        std::string map_frame_;
        std::string odom_frame_;
        std::string base_frame_;

        robot_sdk::pb::NavigationCmd      pb_cmd_;
        zsibot::rmw::Subscriber<NavState> sub_;
        zsibot::rmw::Publisher<NavCmd>    pub_;
    };
}  // namespace jszr_tf
#endif
