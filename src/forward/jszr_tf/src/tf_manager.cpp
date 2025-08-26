#include "gazebo_tf.h"
#include "localization_tf.h"
#include "mc_tf.h"
#include "mujoco_tf.h"

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace jszr_tf
{
    class TFManager : public rclcpp::Node
    {
    public:
        TFManager()
            : Node("tf_manager")
        {
            this->declare_parameter<std::string>("tf_type", "localization_tf");
            std::string tf_type;
            this->get_parameter("tf_type", tf_type_);
        }

        void initializeTFInstance()
        {
            init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0),
                [this]()
                {
                    init_timer_->cancel();
                    if (tf_type_ == "localization_tf")
                    {
                        RCLCPP_INFO(this->get_logger(), "using localization_tf");
                        tf_instance_ = std::make_unique<jszr_tf::LocalizationTF>(shared_from_this());
                    }
                    else if (tf_type_ == "gazebo_tf")
                    {
                        RCLCPP_INFO(this->get_logger(), "using gazebo_tf");
                        tf_instance_ = std::make_unique<jszr_tf::GazeboTF>(shared_from_this());
                    }
                    else if (tf_type_ == "mc_tf")
                    {
                        RCLCPP_INFO(this->get_logger(), "using mc_tf");
                        tf_instance_ = std::make_unique<jszr_tf::MCTF>(shared_from_this());
                    }
                    else if (tf_type_ == "mujoco_tf")
                    {
                        RCLCPP_INFO(this->get_logger(), "using mujoco_tf");
                        tf_instance_ = std::make_unique<jszr_tf::MujocoTF>(shared_from_this());
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid tf_type: %s", tf_type_.c_str());
                        rclcpp::shutdown();
                    }
                });
        }

    private:
        std::string                      tf_type_;
        std::unique_ptr<jszr_tf::BaseTF> tf_instance_;
        rclcpp::TimerBase::SharedPtr     init_timer_;
    };
}  // namespace jszr_tf

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<jszr_tf::TFManager>();
    node->initializeTFInstance();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
