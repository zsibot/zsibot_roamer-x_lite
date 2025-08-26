#ifndef SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
#define SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo_ros/node.hpp>
#include "livox_ode_multiray_shape.h"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "robots_dog_msgs/msg/custom_msg.hpp"

namespace gazebo
{

struct AviaRotateInfo {
        double time;
        double azimuth;
        double zenith;
        std::uint8_t line;
};

class LivoxPointsPlugin : public RayPlugin {
        public:
        LivoxPointsPlugin ();
        virtual ~LivoxPointsPlugin ();
        void Load (sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

        private:
        ignition::math::Angle AngleMin () const;
        ignition::math::Angle AngleMax () const;
        double AngleResolution () const;
        double RangeMin () const;
        double RangeMax () const;
        double RangeResolution () const;
        int RayCount () const;
        int RangeCount () const;
        int VerticalRayCount () const;
        int VerticalRangeCount () const;
        ignition::math::Angle VerticalAngleMin () const;
        ignition::math::Angle VerticalAngleMax () const;
        double VerticalAngleResolution () const;

        protected:
        virtual void OnNewLaserScans ();

        private:
        void InitializeRays (std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                            boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape);  // 새 코드의 shared_ptr 사용
        void InitializeScan (msgs::LaserScan*& scan);
        void SendRosTf (const ignition::math::Pose3d& pose, const std::string& father_frame, const std::string& child_frame);
        void PublishPointCloud (std::vector<std::pair<int, AviaRotateInfo>>& points_pair);
        void PublishPointCloud2XYZ (std::vector<std::pair<int, AviaRotateInfo>>& points_pair);
        void PublishLivoxROSDriverCustomMsg (std::vector<std::pair<int, AviaRotateInfo>>& points_pair);
        void PublishPointCloud2XYZRTLT (std::vector<std::pair<int, AviaRotateInfo>>& points_pair);

        boost::shared_ptr<physics::LivoxOdeMultiRayShape> rayShape;
        gazebo::physics::CollisionPtr laserCollision;
        physics::EntityPtr parentEntity;
        transport::PublisherPtr scanPub;
        sdf::ElementPtr sdfPtr;
        msgs::LaserScanStamped laserMsg;
        transport::NodePtr node;
        gazebo::sensors::SensorPtr raySensor;
        std::vector<AviaRotateInfo> aviaInfos;

        rclcpp::Node::SharedPtr rosNode;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rosPointPub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_pub;  // 새 코드에서 추가
        rclcpp::Publisher<robots_dog_msgs::msg::CustomMsg>::SharedPtr custom_pub;  // 새 코드에서 추가
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
        gazebo_ros::Node::SharedPtr node_;

        std::string parent_name;  // 새 코드에서 추가
        std::string child_name;  // 새 코드에서 추가

        int64_t samplesStep = 0;
        int64_t currStartIndex = 0;
        int64_t maxPointSize = 1000;
        int64_t downSample = 1;
        uint16_t publishPointCloudType;
        bool visualize = false;
        std::string frameName = "livox";

        double maxDist = 400.0;
        double minDist = 0.1;

        bool useInf = true;
};

}  // namespace gazebo

#endif  // SRC_GAZEBO_LIVOX_POINTS_PLUGIN_H
