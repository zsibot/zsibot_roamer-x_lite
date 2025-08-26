/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <string>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace gazebo
{
    class UnitreeFootContactPlugin : public SensorPlugin
    {
        public:
        UnitreeFootContactPlugin() : SensorPlugin(){}
        ~UnitreeFootContactPlugin(){}

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor); // Make sure the parent sensor is valid.        
            if (!this->parentSensor){
                gzerr << "UnitreeFootContactPlugin requires a ContactSensor.\n";
                return;
            }
            this->rosnode = gazebo_ros::Node::Get (_sdf);
            // add "visual" is for the same name of draw node
            this->force_pub = this->rosnode->create_publisher<geometry_msgs::msg::WrenchStamped>("/visual/"+_sensor->Name()+"/the_force", 100);
            // Connect to the sensor update event.
            this->update_connection = this->parentSensor->ConnectUpdated(std::bind(&UnitreeFootContactPlugin::OnUpdate, this));
            this->parentSensor->SetActive(true); // Make sure the parent sensor is active.
            count = 0;
            Fx = 0;
            Fy = 0;
            Fz = 0;
            RCLCPP_INFO(rclcpp::get_logger("UnitreeFootContactPlugin"),"Load %s plugin.", _sensor->Name().c_str());
        }

        private:
        void OnUpdate()
        {
            msgs::Contacts contacts = this->parentSensor->Contacts();
            count = contacts.contact_size();
            // std::cout << count <<"\n";
            for (unsigned int i = 0; i < count; ++i){
                if(contacts.contact(i).position_size() != 1){
                    RCLCPP_ERROR(this->rosnode->get_logger(),"Contact count isn't correct!!!!");
                }     
                for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){                 
                    // std::cout << i <<" "<< contacts.contact(i).position_size() <<" Force:"
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().x() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().y() << " "
                    //           << contacts.contact(i).wrench(j).body_1_wrench().force().z() << "\n";
                    Fx += contacts.contact(i).wrench(0).body_1_wrench().force().x(); // Notice: the force is in local coordinate, not in world or base coordnate.
                    Fy += contacts.contact(i).wrench(0).body_1_wrench().force().y();
                    Fz += contacts.contact(i).wrench(0).body_1_wrench().force().z();
                }
            }
            geometry_msgs::msg::WrenchStamped force;
            if(count != 0){           
                force.wrench.force.x = Fx/double(count);
                force.wrench.force.y = Fy/double(count);
                force.wrench.force.z = Fz/double(count);
                count = 0;
                Fx = 0;
                Fy = 0;
                Fz = 0;
            }
            else{
                force.wrench.force.x = 0;
                force.wrench.force.y = 0;
                force.wrench.force.z = 0;
            }
            this->force_pub->publish(force);
        }

        private:
            rclcpp::Node::SharedPtr rosnode;
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub;
            event::ConnectionPtr update_connection;
            sensors::ContactSensorPtr parentSensor;      
            int count = 0;
            double Fx=0, Fy=0, Fz=0;
    };
    GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}
    