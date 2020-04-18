/** @file odometry publisher
*  @version 1.0
*  @date 3/10/2020
*
*  @brief
*  Publishing odometry information for navigation stack
*
*/

#include "odom.h"
#include "dji_sdk/dji_sdk.h"

// global variables for subscribed topics
geometry_msgs::PointStamped local_position;
geometry_msgs::Vector3Stamped current_velocity;
geometry_msgs::QuaternionStamped current_attitude;
sensor_msgs::Imu current_imu;


int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber attitudeSub = nh.subscribe("dji_sdk/attitude", 100, &attitude_callback);
  ros::Subscriber velocitySub = nh.subscribe("dji_sdk/velocity", 100, &velocity_callback);
  ros::Subscriber imuSub = nh.subscribe("dji_sdk/imu", 100, &imu_callback);
  ros::Subscriber localPosition = nh.subscribe("dji_sdk/local_position", 100, &local_position_callback);

  //Publishers
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  tf::TransformBroadcaster odom_broadcaster;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  ros::Rate r(200.0);

  while(nh.ok()){
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    geometry_msgs::Quaternion odom_quat = current_attitude.quaternion;
    extract_yaw(odom_quat);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_stabilized";

    odom_trans.transform.translation.x = local_position.point.x;
    odom_trans.transform.translation.y = local_position.point.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = local_position.point.x;
    odom.pose.pose.position.y = local_position.point.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_stabilized";
    odom.twist.twist.linear.x = current_velocity.vector.x;
    odom.twist.twist.linear.y = current_velocity.vector.y;
    odom.twist.twist.angular.z = current_imu.angular_velocity.z;
    //ROS_INFO_STREAM(odom);
    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
  return 0;
}

void extract_yaw(geometry_msgs::Quaternion& q){
  q.x = 0;
  q.y = 0;
  double mag = sqrt(q.w*q.w + q.z*q.z);
  q.w /= mag;
  q.z /= mag;
}


void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  current_attitude = *msg;
}

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
  current_velocity = *msg;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  current_imu = *msg;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg) {
  local_position = *msg;
}
