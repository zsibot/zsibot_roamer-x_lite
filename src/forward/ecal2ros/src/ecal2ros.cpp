#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.h>
#include <zsibot/base.hpp>
#include <zsibot/param.hpp>
#include <zsibot/rmw/ecal.hpp>
#include <zsibot_pb_msg/sensor/image.pb.h>

struct MyParam
{
    std::string ros_topic_name{ "/image_raw/compressed" };
    std::string ecal_iamge_topic_name{ "image_ecal" };
    std::string frame_id{ "camera_frame" };
    std::string image_format{ "jpeg" };
};

ZSIBOT_BINDING_OBJECT(MyParam, ros_topic_name, ecal_iamge_topic_name, frame_id, image_format);
ZSIBOT_PARAM("ecal2ros", MyParam, ecal2rosParam);


ZSIBOT_TOPIC(zsibot_msg::Image, ImageECAL);
ZSIBOT_IMPL_TOPIC(ImageECAL, Ecal);

class TransferManager : public rclcpp::Node
{
public:
    TransferManager()
        : Node("ecal2ros2")
    {

        const bool ret = ecal2rosParam::read(param_);
        if (not ret)
        {
            throw std::runtime_error("ecal2ros2 read param error!");
        }

        image_sub_ecal_ptr_ = std::make_shared<zsibot::rmw::Subscriber<ImageECAL>>();
        ZSIBOT_UNUSED(image_sub_ecal_ptr_->init(param_.ecal_iamge_topic_name));
        publisher_image_ptr_ =
            this->create_publisher<sensor_msgs::msg::CompressedImage>(param_.ros_topic_name, rclcpp::QoS(1).best_effort());

        image_sub_ecal_ptr_->subscribe(
            [this](const zsibot::rmw::MessagePtr<zsibot_msg::Image>& msg)
            {
                sensor_msgs::msg::CompressedImage image_msg;
                image_msg.header.stamp.sec     = msg->data.header().sec();
                image_msg.header.stamp.nanosec = msg->data.header().nsec();
                image_msg.header.frame_id      = param_.frame_id;
                image_msg.format               = param_.image_format;
                image_msg.data.assign(msg->data.data().begin(), msg->data.data().end());
                publisher_image_ptr_->publish(image_msg);
            });
        RCLCPP_INFO(rclcpp::get_logger("ecal2ros"), "ecal2ros2 init OK");
        RCLCPP_INFO(rclcpp::get_logger("ecal2ros"), "ros topic name  = %s", param_.ros_topic_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("ecal2ros"), "ecal topic name = %s", param_.ecal_iamge_topic_name.c_str());
        RCLCPP_INFO(rclcpp::get_logger("ecal2ros"), "image format    = %s", param_.image_format.c_str());
    }

private:
    std::shared_ptr<zsibot::rmw::Subscriber<ImageECAL>>             image_sub_ecal_ptr_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_image_ptr_;
    std::string                                                     topic_name_;
    std::string                                                     image_format_;
    MyParam                                                         param_;
};

int main(int argc, char* argv[])
{
    zsibot::initialize();
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransferManager>());
    rclcpp::shutdown();
    zsibot::finalize();
    return 0;
}
