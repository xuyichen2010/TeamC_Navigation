#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <gazebo_msgs/ContactsState.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>

#include <cmath>
#include <string>


#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif


namespace gazebo
{
    class DJI_ROS_ControlPlugin : public ModelPlugin{
        public:
            DJI_ROS_ControlPlugin(){}
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
                std::cout<<"\033[1;32m DJI ROS Control Plugin is successfully plugged in to "<<_model->GetName()<<"\033[0m\n";
                this->model = _model;
                this->world = _model->GetWorld();
                this->base_link = model->GetLink();
                std::string ns = _model->GetName();

                ros::SubscribeOptions attitude_ops = ros::SubscribeOptions::create<geometry_msgs::QuaternionStamped>(
                    "/dji_sdk/attitude", 1000,
                    boost::bind(&DJI_ROS_ControlPlugin::attitude_callback, this, _1),
                    ros::VoidPtr(), &this->callback_queue);
                this->attitude_subscriber = nh.subscribe(attitude_ops);

                ros::SubscribeOptions local_position_ops = ros::SubscribeOptions::create<geometry_msgs::PointStamped>(
                    "/dji_sdk/local_position",1000,
                    boost::bind(&DJI_ROS_ControlPlugin::local_position_callback,this,_1),
                    ros::VoidPtr(),&this->callback_queue);
                this->local_position_subscriber = nh.subscribe(local_position_ops);

                this->reset();
                this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DJI_ROS_ControlPlugin::OnUpdate,this));
            }
            void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg)
            {
                #if GAZEBO_MAJOR_VERSION >= 8
                    this->base_orientation.W() = quat_msg->quaternion.w;
                    this->base_orientation.X() = quat_msg->quaternion.x;
                    this->base_orientation.Y() = quat_msg->quaternion.y;
                    this->base_orientation.Z() = quat_msg->quaternion.z;
                #else
                    this->base_orientation.w = quat_msg->quaternion.w;
                    this->base_orientation.x = quat_msg->quaternion.x;
                    this->base_orientation.y = quat_msg->quaternion.y;
                    this->base_orientation.z = quat_msg->quaternion.z;
                #endif
            }
            void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
            {
                #if GAZEBO_MAJOR_VERSION >= 8
                   this->base_position.Set(position_msg->point.X(),position_msg->point.Y(),position_msg->point.Z());
                #else
                    this->base_position.Set(position_msg->point.x,position_msg->point.y,position_msg->point.z);
                #endif
            }
            void reset()
            {
                this->base_link->SetForce(math::Vector3(0,0,0));
                this->base_link->SetTorque(math::Vector3(0,0,0));
                this->base_orientation.Set(1,0,0,0);
                double x = this->base_link->GetWorldPose().Ign().Pos().X();
                double y = this->base_link->GetWorldPose().Ign().Pos().Y();
                double z = this->base_link->GetWorldPose().Ign().Pos().Z();
                this->base_position.Set(x,y,z);
            }

            void OnUpdate(){
                this->callback_queue.callAvailable();
                this->base_orientation.Normalize();
                #if GAZEBO_MAJOR_VERSION >= 8
                    math::Pose3d target_pose;
                #else
                    math::Pose target_pose;
                #endif
                target_pose.Set(this->base_position,this->base_orientation);
                // ROS_INFO_STREAM(target_pose);
                this->world->SetPaused(false);
                this->model->SetWorldPose(target_pose);
                this->model->SetLinearVel(math::Vector3(0,0,0));
                this->model->SetAngularVel(math::Vector3(0,0,0));
            }
        private:
            ros::NodeHandle nh;
            physics::WorldPtr world;
            physics::ModelPtr model;
            physics::LinkPtr base_link;
            event::ConnectionPtr update_connection;

            #if (GAZEBO_MAJOR_VERSION >= 8)
                math::Vector3d base_position;
                math::Quaterniond base_orientation;
            #else
                math::Vector3 base_position;
                math::Quaternion base_orientation;
            #endif
            ros::Subscriber attitude_subscriber;
            ros::Subscriber local_position_subscriber;

            double initial_height = 0;
            ros::CallbackQueue callback_queue;
    };
    GZ_REGISTER_MODEL_PLUGIN(DJI_ROS_ControlPlugin)
}
