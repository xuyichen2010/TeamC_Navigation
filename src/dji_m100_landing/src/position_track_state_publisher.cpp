#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <string>

#include <geometry_msgs/PointStamped.h>

ros::Subscriber local_position_sub;

ros::Publisher x_state_pub;
ros::Publisher y_state_pub;
ros::Publisher z_state_pub;

double local_x_state;
double local_y_state;
double local_z_state;

void localPosCallback(const geometry_msgs::PointStamped::ConstPtr& local_pos_msg)
{
	local_x_state = local_pos_msg->point.x;
	local_y_state = local_pos_msg->point.y;
	local_z_state = local_pos_msg->point.z;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "position_track_state_publisher");
	ROS_INFO("Starting position publisher");

	ros::NodeHandle nh;

	local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPosCallback);

	x_state_pub = nh.advertise<std_msgs::Float64>("/position_track/x_state", 10);
	y_state_pub = nh.advertise<std_msgs::Float64>("/position_track/y_state", 10);
	z_state_pub = nh.advertise<std_msgs::Float64>("/position_track/z_state", 10);

	std_msgs::Float64 x_state_msg;
	std_msgs::Float64 y_state_msg;
	std_msgs::Float64 z_state_msg;

	ros::Rate loop_rate(200);

	while(ros::ok())
	{
		ros::spinOnce();

		x_state_msg.data = local_x_state;
		y_state_msg.data = local_y_state;
		z_state_msg.data = local_z_state;

		x_state_pub.publish(x_state_msg); 
		y_state_pub.publish(y_state_msg); 
		z_state_pub.publish(z_state_msg); 

		loop_rate.sleep();
	}








}