/** @file demo_local_position_control.h
 *  @version 3.3
 *  @date September, 2017
 *
 *  @brief
 *  demo sample of how to use local position control APIs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/SetLocalPosRef.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/NavSatFix.h>
//DJI SDK includes
#include <dji_sdk/DroneTaskControl.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

bool set_local_position();

void extract_yaw(geometry_msgs::Quaternion& q);

void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);

void velocity_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg);
