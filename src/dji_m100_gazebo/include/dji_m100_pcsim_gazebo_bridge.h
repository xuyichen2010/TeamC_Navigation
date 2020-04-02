#include <ros/ros.h>
#include <ros/console.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>

#include <cmath>
#include <gazebo/common/common.hh>
#if GAZEBO_MAJOR_VERSION >= 8
namespace math = ignition::math;
#else
namespace math = gazebo::math;
#endif

#include <string>
std::string model_name = "dji_m100";
std::string reference_frame = "world";

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg);
void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity_msg);
void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& position_msg);
bool obtain_control();
bool set_local_position();