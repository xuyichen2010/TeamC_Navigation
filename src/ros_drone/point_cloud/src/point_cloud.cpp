/** @file odometry publisher
*  @version 1.0
*  @date 3/10/2020
*
*  @brief
*  Publishing odometry information for navigation stack
*
*/

#include "point_cloud.h"

// global variables for subscribed topics
sensor_msgs::PointCloud2 input_point_cloud;
int recieved = 0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_node");
  ros::NodeHandle nh;
  // Subscribe to messages from dji_sdk_node
  ros::Subscriber cloud_sub = nh.subscribe("/guidance_rear/points2", 10, &cloud_callback);

  //Publishers
  ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud>("cloud", 10);
  ros::Rate r(1.0);

  while(nh.ok()){
    ros::spinOnce();
    sensor_msgs::PointCloud cloud;
    if (!recieved){
      ROS_INFO("Empty Point Cloud!!!\n");
      continue;
    }
    if (!sensor_msgs::convertPointCloud2ToPointCloud(input_point_cloud, cloud)){
      ROS_INFO("convert to point cloud 1 FAIL!!!\n");
    }
    cloud.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud);
    ROS_INFO_STREAM(cloud);
    r.sleep();
  }
  return 0;
}

void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  recieved = 1;
  input_point_cloud = *msg;
}
