#include "dji_m100_pcsim_gazebo_bridge.h"

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

geometry_msgs::Pose target_pose;
geometry_msgs::Twist target_twist;

gazebo_msgs::ModelState target_model_state;
gazebo_msgs::SetModelState set_model_state;

ros::ServiceClient model_state_client;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient set_local_pos_reference;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "hku_m100_pcsim_gazebo_bridge");

  ros::NodeHandle nh;

  ros::Subscriber attitude_subscriber 			= nh.subscribe("/dji_sdk/attitude", 1000, &attitude_callback);
  ros::Subscriber velocity_subscriber 			= nh.subscribe("/dji_sdk/velocity", 1000, &velocity_callback);
  ros::Subscriber local_position_subscriber	 	= nh.subscribe("/dji_sdk/local_position", 1000, &local_position_callback);

  model_state_client		 					= nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state", true);
  sdk_ctrl_authority_service 					= nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
  set_local_pos_reference    					= nh.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");

  bool obtain_control_result = obtain_control();
  if (!set_local_position()) // We need this for height
  {
    ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    return 1;
  }

  ROS_INFO("Bridge between PC sim and gazebo connected");

  // ros::Rate spin_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();

    if(model_state_client)
    {
      target_model_state.model_name = model_name;
      target_model_state.reference_frame = reference_frame;
      target_model_state.pose = target_pose;
      target_model_state.twist = target_twist;
      set_model_state.request.model_state = target_model_state;
      model_state_client.call(set_model_state);
    }
    else
    {
      // std::cout << "connection with service lost!!" << std::endl;
      ROS_ERROR("No gazebo model detected.");
    }

    // spin_rate.sleep();

  }
  ros::spin();
  return 0;
}

bool set_local_position()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  set_local_pos_reference.call(localPosReferenceSetter);
  return localPosReferenceSetter.response.result;
}

bool obtain_control()
{
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  sdk_ctrl_authority_service.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& quat_msg){
  // Quaternion for the rotation from FLU body frame to ENU ground.
    target_pose.orientation.w = quat_msg->quaternion.w;
    target_pose.orientation.x = quat_msg->quaternion.x;
    target_pose.orientation.y = quat_msg->quaternion.y;
    target_pose.orientation.z = quat_msg->quaternion.z;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& velocity_msg){
  // Velocity in ENU ground frame
  target_twist.linear.x  = velocity_msg->vector.x;
  target_twist.linear.y  = velocity_msg->vector.y;
  target_twist.linear.z  = velocity_msg->vector.z;
  // std::cout << "velocity callback get called" << std::endl;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& position_msg){
  // Local position in Cartesian ENU frame
  target_pose.position.x = position_msg->point.x;
  target_pose.position.y = position_msg->point.y;
  // target_pose.position.y = -position_msg->point.y;
  target_pose.position.z = position_msg->point.z;
  // std::cout << "local position callback get called" << std::endl;
  }
