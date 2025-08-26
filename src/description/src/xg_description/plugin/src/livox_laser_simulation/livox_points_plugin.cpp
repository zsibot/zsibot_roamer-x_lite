#include <rclcpp/rclcpp.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/logging.hpp>
// #include <random>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <chrono>
#include "livox_laser_simulation/livox_points_plugin.h"
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "robots_dog_msgs/msg/custom_msg.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "livox_laser_simulation/livox_point_xyzrtl.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace gazebo
{

GZ_REGISTER_SENSOR_PLUGIN (LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin () : tfBroadcaster (std::make_shared<tf2_ros::TransformBroadcaster> (gazebo_ros::Node::Get ())) {}

LivoxPointsPlugin::~LivoxPointsPlugin () {}

void convertDataToRotateInfo (const std::vector<std::vector<double>>& datas, std::vector<AviaRotateInfo>& avia_infos)
{
        avia_infos.reserve (datas.size ());
        double deg_2_rad = M_PI / 180.0;
        for (auto& data : datas)
        {
                if (data.size () == 3)
                {
                        avia_infos.emplace_back ();
                        avia_infos.back ().time = data [0];
                        avia_infos.back ().azimuth = data [1] * deg_2_rad;
                        avia_infos.back ().zenith = data [2] * deg_2_rad - M_PI_2;
                }
                else
                {
                        RCLCPP_ERROR (rclcpp::get_logger ("convertDataToRotateInfo"), "data size is not 3!");
                }
        }
}

void LivoxPointsPlugin::Load (gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
{
        // Gazebo ROS
        node_ = gazebo_ros::Node::Get (sdf);

        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string> ("csv_file_name");
        RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "load csv file name: %s", file_name.c_str ());
        if (!CsvReader::ReadCsvFile (file_name, datas))
        {
                RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "cannot get csv file! %s will return !", file_name.c_str ());
                return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement ("ray");
        auto scanElem = rayElem->GetElement ("scan");
        auto rangeElem = rayElem->GetElement ("range");

        raySensor = _parent;
        auto sensor_pose = raySensor->Pose ();
        auto curr_scan_topic = sdf->Get<std::string> ("topic");
        RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "ros topic name: %s", curr_scan_topic.c_str ());

        node = transport::NodePtr (new transport::Node ());
        node->Init (raySensor->WorldName ());

        // PointCloud2 publisher
        cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2> (curr_scan_topic + "_PointCloud2", 10);
        // CustomMsg publisher
        custom_pub = node_->create_publisher<robots_dog_msgs::msg::CustomMsg> (curr_scan_topic + "_CustomMsg", 10);

        scanPub = node->Advertise<msgs::LaserScanStamped> (curr_scan_topic + "laserscan", 50);

        aviaInfos.clear ();
        convertDataToRotateInfo (datas, aviaInfos);
        RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "scan info size: %ld", aviaInfos.size ());
        maxPointSize = aviaInfos.size ();

        RayPlugin::Load (_parent, sdfPtr);
        laserMsg.mutable_scan ()->set_frame (_parent->ParentName ());
        parentEntity = this->world->EntityByName (_parent->ParentName ());
        auto physics = world->Physics ();
        laserCollision = physics->CreateCollision ("multiray", _parent->ParentName ());
        laserCollision->SetName ("ray_sensor_collision");
        laserCollision->SetRelativePose (_parent->Pose ());
        laserCollision->SetInitialRelativePose (_parent->Pose ());
        rayShape.reset (new gazebo::physics::LivoxOdeMultiRayShape (laserCollision));
        laserCollision->SetShape (boost::dynamic_pointer_cast<gazebo::physics::Shape>(rayShape));  // boost::shared_ptr 사용
        samplesStep = sdfPtr->Get<int> ("samples");
        downSample = sdfPtr->Get<int> ("downsample");
        if (downSample < 1)
        {
                downSample = 1;
        }
        RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "sample: %ld", samplesStep);
        RCLCPP_INFO (rclcpp::get_logger ("LivoxPointsPlugin"), "downsample: %ld", downSample);
        rayShape->RayShapes ().reserve (samplesStep / downSample);
        rayShape->Load (sdfPtr);
        rayShape->Init ();
        minDist = rangeElem->Get<double> ("min");
        maxDist = rangeElem->Get<double> ("max");
        auto offset = laserCollision->RelativePose ();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
                int index = j % maxPointSize;
                auto& rotate_info = aviaInfos [index];
                ignition::math::Quaterniond ray;
                ray.Euler (ignition::math::Vector3d (0.0, rotate_info.zenith, rotate_info.azimuth));
                auto axis = offset.Rot () * ray * ignition::math::Vector3d (1.0, 0.0, 0.0);
                start_point = minDist * axis + offset.Pos ();
                end_point = maxDist * axis + offset.Pos ();
                rayShape->AddRay (start_point, end_point);
        }
}

void LivoxPointsPlugin::OnNewLaserScans ()
{
        if (rayShape)
        {
                std::vector<std::pair<int, AviaRotateInfo>> points_pair;
                InitializeRays (points_pair, rayShape);
                rayShape->Update ();

                msgs::Set (laserMsg.mutable_time (), world->SimTime ());
                msgs::LaserScan* scan = laserMsg.mutable_scan ();
                InitializeScan (scan);

                robots_dog_msgs::msg::CustomMsg pp_livox;
                pp_livox.header.stamp = node_->get_clock ()->now ();
                pp_livox.header.frame_id = raySensor->Name ();
                int count = 0;
                auto start_time = std::chrono::high_resolution_clock::now ();

                sensor_msgs::msg::PointCloud2 scan_point;
                pcl::PointCloud<pcl::PointXYZI> pc;
                pc.points.resize (points_pair.size ());
                int pt_count = 0;
                // std::random_device rd; // 获得种子
                // std::mt19937 gen (rd ());
                // std::uniform_int_distribution<> distrib (0, 100);
                for (auto& pair : points_pair)
                {
                        auto range = rayShape->GetRange (pair.first);
                        // auto intensity = rayShape->GetRetro (pair.first);

                        if (range >= RangeMax ())
                        {
                                range = 0;
                        }
                        else if (range <= RangeMin ())
                        {
                                range = 0;
                        }

                        auto rotate_info = pair.second;
                        ignition::math::Quaterniond ray;
                        ray.Euler (ignition::math::Vector3d (0.0, rotate_info.zenith, rotate_info.azimuth));
                        auto axis = ray * ignition::math::Vector3d (1.0, 0.0, 0.0);
                        auto point = range * axis;

                        pcl::PointXYZI pt;
                        pt.x = point.X ();
                        pt.y = point.Y ();
                        pt.z = point.Z ();
                        // pt.intensity = distrib (gen);
                        pt.intensity = (range - RangeMin ()) / RangeMax () * 255;
                        pc [pt_count++] = pt;

                        robots_dog_msgs::msg::CustomPoint p;
                        p.x = point.X ();
                        p.y = point.Y ();
                        p.z = point.Z ();
                        p.reflectivity = (range - RangeMin ()) / RangeMax () * 255;

                        auto end_time = std::chrono::high_resolution_clock::now ();
                        auto elapsed_time = std::chrono::duration_cast<std::chrono::nanoseconds>(end_time - start_time).count ();
                        p.offset_time = elapsed_time;

                        pp_livox.points.push_back (p);
                        count++;
                }

                pp_livox.point_num = count;
                custom_pub->publish (pp_livox);

                pcl::toROSMsg (pc, scan_point);
                scan_point.header.stamp = node_->get_clock ()->now ();
                scan_point.header.frame_id = raySensor->Name ();
                cloud2_pub->publish (scan_point);

                //                // TF Publish
                //                geometry_msgs::msg::TransformStamped transformStamped;
                //                transformStamped.header.stamp = node_->get_clock ()->now ();
                //                transformStamped.header.frame_id = "world";  // parent frame
                //                transformStamped.child_frame_id = raySensor->Name ();  // child frame
                //                transformStamped.transform.translation.x = raySensor->Pose ().Pos ().X ();
                //                transformStamped.transform.translation.y = raySensor->Pose ().Pos ().Y ();
                //                transformStamped.transform.translation.z = raySensor->Pose ().Pos ().Z ();
                //                transformStamped.transform.rotation.x = raySensor->Pose ().Rot ().X ();
                //                transformStamped.transform.rotation.y = raySensor->Pose ().Rot ().Y ();
                //                transformStamped.transform.rotation.z = raySensor->Pose ().Rot ().Z ();
                //                transformStamped.transform.rotation.w = raySensor->Pose ().Rot ().W ();
                //
                //                tfBroadcaster->sendTransform (transformStamped);

                // if (scanPub && scanPub->HasConnections ())
                //         scanPub->Publish (laserMsg);
        }
}

void LivoxPointsPlugin::InitializeRays (std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
                                       boost::shared_ptr<gazebo::physics::LivoxOdeMultiRayShape>& ray_shape)  // boost::shared_ptr 사용
{
        auto& rays = ray_shape->RayShapes ();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose ();
        int64_t end_index = currStartIndex + samplesStep;
        long unsigned int ray_index = 0;
        auto ray_size = rays.size ();
        points_pair.reserve (rays.size ());
        for (int k = currStartIndex; k < end_index; k += downSample)
        {
                auto index = k % maxPointSize;
                auto& rotate_info = aviaInfos [index];
                ray.Euler (ignition::math::Vector3d (0.0, rotate_info.zenith, rotate_info.azimuth));
                auto axis = offset.Rot () * ray * ignition::math::Vector3d (1.0, 0.0, 0.0);
                start_point = minDist * axis + offset.Pos ();
                end_point = maxDist * axis + offset.Pos ();
                if (ray_index < ray_size)
                {
                        rays [ray_index]->SetPoints (start_point, end_point);
                        points_pair.emplace_back (ray_index, rotate_info);
                }
                ray_index++;
        }
        currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan (msgs::LaserScan*& scan)
{
        msgs::Set (scan->mutable_world_pose (), raySensor->Pose () + parentEntity->WorldPose ());
        scan->set_angle_min (AngleMin ().Radian ());
        scan->set_angle_max (AngleMax ().Radian ());
        scan->set_angle_step (AngleResolution ());
        scan->set_count (RangeCount ());

        scan->set_vertical_angle_min (VerticalAngleMin ().Radian ());
        scan->set_vertical_angle_max (VerticalAngleMax ().Radian ());
        scan->set_vertical_angle_step (VerticalAngleResolution ());
        scan->set_vertical_count (VerticalRangeCount ());

        scan->set_range_min (RangeMin ());
        scan->set_range_max (RangeMax ());

        scan->clear_ranges ();
        scan->clear_intensities ();

        unsigned int rangeCount = RangeCount ();
        unsigned int verticalRangeCount = VerticalRangeCount ();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
                for (unsigned int i = 0; i < rangeCount; ++i)
                {
                        scan->add_ranges (0);
                        scan->add_intensities (0);
                }
        }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin () const
{
        if (rayShape)
                return rayShape->MinAngle ();
        else
                return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax () const
{
        if (rayShape)
        {
                return ignition::math::Angle (rayShape->MaxAngle ().Radian ());
        }
        else
                return -1;
}

double LivoxPointsPlugin::RangeMin () const
{
        if (rayShape)
                return rayShape->GetMinRange ();
        else
                return -1;
}

double LivoxPointsPlugin::RangeMax () const
{
        if (rayShape)
                return rayShape->GetMaxRange ();
        else
                return -1;
}

double LivoxPointsPlugin::AngleResolution () const { return (AngleMax () - AngleMin ()).Radian () / (RangeCount () - 1); }

double LivoxPointsPlugin::RangeResolution () const
{
        if (rayShape)
                return rayShape->GetResRange ();
        else
                return -1;
}

int LivoxPointsPlugin::RayCount () const
{
        if (rayShape)
                return rayShape->GetSampleCount ();
        else
                return -1;
}

int LivoxPointsPlugin::RangeCount () const
{
        if (rayShape)
                return rayShape->GetSampleCount () * rayShape->GetScanResolution ();
        else
                return -1;
}

int LivoxPointsPlugin::VerticalRayCount () const
{
        if (rayShape)
                return rayShape->GetVerticalSampleCount ();
        else
                return -1;
}

int LivoxPointsPlugin::VerticalRangeCount () const
{
        if (rayShape)
                return rayShape->GetVerticalSampleCount () * rayShape->GetVerticalScanResolution ();
        else
                return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin () const
{
        if (rayShape)
        {
                return ignition::math::Angle (rayShape->VerticalMinAngle ().Radian ());
        }
        else
                return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax () const
{
        if (rayShape)
        {
                return ignition::math::Angle (rayShape->VerticalMaxAngle ().Radian ());
        }
        else
                return -1;
}

double LivoxPointsPlugin::VerticalAngleResolution () const
{
        return (VerticalAngleMax () - VerticalAngleMin ()).Radian () / (VerticalRangeCount () - 1);
}

} // namespace gazebo