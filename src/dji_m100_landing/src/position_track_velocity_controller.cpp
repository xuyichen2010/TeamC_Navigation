#include <ros/ros.h>
#include <ros/console.h>

#include <vector>
#include <iterator>
#include <string>

#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneTaskControl.h>
// #include <dji_sdk/GimbalAngleControl.h>
// #include <dji_sdk/GimbalSpeedControl.h>
// #include <dji_sdk/VelocityControl.h>
#include <sensor_msgs/Joy.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <iostream>

#include <Eigen/Geometry> 

ros::Subscriber velocity_control_x_sub;
ros::Subscriber velocity_control_y_sub;
ros::Subscriber velocity_control_yaw_sub;
ros::Subscriber position_track_enable_sub;
ros::Subscriber landing_condition_met_sub;
ros::Subscriber relanding_condition_met_sub;

ros::Publisher ctrlPosYawPub;

// ros::ServiceClient velocity_control_service;
ros::ServiceClient sdk_permission_control_service;
ros::ServiceClient drone_task_service;

double velocity_control_effort_x;
double velocity_control_effort_y;
double velocity_control_effort_yaw;

const double descending_speed = -0.5;
const double ascending_speed = 0.5;

bool position_track_enabled = false;
bool landing_condition_met = false;
bool relanding_condition_met = false;

std::string topic_from_controller;

void velocityControlEffortXCallback(std_msgs::Float64 velocity_control_effort_x_msg)
{
	velocity_control_effort_x = velocity_control_effort_x_msg.data;
}

void velocityControlEffortYCallback(std_msgs::Float64 velocity_control_effort_y_msg)
{
	velocity_control_effort_y = velocity_control_effort_y_msg.data;
}

void velocityControlEffortYawCallback(std_msgs::Float64 velocity_control_effort_yaw_msg)
{
	velocity_control_effort_yaw = velocity_control_effort_yaw_msg.data;
}

void positionTrackEnableCallback(const std_msgs::Bool& position_track_enable_msg)
{
  position_track_enabled = position_track_enable_msg.data;
}

void landingConditionMetCallback(const std_msgs::Bool& landing_condition_met_msg)
{
	landing_condition_met = landing_condition_met_msg.data;
}

void relandingConditionMetCallback(const std_msgs::Bool& relanding_condition_met_msg)
{
	relanding_condition_met = relanding_condition_met_msg.data;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "position_track_controller");
	ros::NodeHandle nh;

	nh.param<std::string>("topic_from_controller", topic_from_controller, "/position_track/velocity_control_effort_x");

	velocity_control_x_sub = nh.subscribe("/position_track/velocity_control_effort_x", 10, velocityControlEffortXCallback);
	velocity_control_y_sub = nh.subscribe("/position_track/velocity_control_effort_y", 10, velocityControlEffortYCallback);
	velocity_control_yaw_sub = nh.subscribe("/position_track/velocity_control_effort_yaw", 10, velocityControlEffortYawCallback);
	position_track_enable_sub = nh.subscribe("/position_track/position_track_enable", 1, positionTrackEnableCallback );
	landing_condition_met_sub = nh.subscribe("/position_track/landing_condition_met", 1, landingConditionMetCallback );   
	relanding_condition_met_sub = nh.subscribe("/position_track/relanding_condition_met", 1, relandingConditionMetCallback );   

	// velocity_control_service = nh.serviceClient<dji_sdk::VelocityControl>("dji_sdk/velocity_control");
	sdk_permission_control_service = nh.serviceClient<dji_sdk::SDKControlAuthority> ("dji_sdk/sdk_control_authority");
	drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
	// dji_sdk::VelocityControl velocity_control;
	dji_sdk::DroneTaskControl droneTaskControl;

	ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);


	int frame = 0;
	double yaw_rate = 0;

	dji_sdk::SDKControlAuthority sdk_permission_control;
	sdk_permission_control.request.control_enable = 1;
	bool control_requested = false;

	while(!(sdk_permission_control_service.call(sdk_permission_control) && sdk_permission_control.response.result))
	{
		ROS_ERROR("Velocity controller: request control failed!");
	}

	while(ros::ok())
	{
		ros::spinOnce();

		if(position_track_enabled)
		{
			if(landing_condition_met)
			{
				ROS_DEBUG_ONCE("Velocity controller: Landing condition met, going down");
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = descending_speed;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;
				droneTaskControl.request.task = 6;

				sensor_msgs::Joy controlPosYaw;
				controlPosYaw.axes.push_back(velocity_control_effort_x);
				controlPosYaw.axes.push_back(velocity_control_effort_y);
				controlPosYaw.axes.push_back(descending_speed);
				controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				ctrlPosYawPub.publish(controlPosYaw);

				if(!(drone_task_service.call(droneTaskControl) && droneTaskControl.response.result))
				{
					ROS_ERROR("takeoff_land fail");
				}
			}
			else if(relanding_condition_met)
			{
				ROS_DEBUG_ONCE("Velocity controller: Relanding condition met, going up");
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = ascending_speed;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;

				// droneTaskControl.request.task = 6;

				sensor_msgs::Joy controlPosYaw;
				controlPosYaw.axes.push_back(velocity_control_effort_x);
				controlPosYaw.axes.push_back(velocity_control_effort_y);
				controlPosYaw.axes.push_back(ascending_speed);
				controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				ctrlPosYawPub.publish(controlPosYaw);

				if(!(drone_task_service.call(droneTaskControl) && droneTaskControl.response.result))
				{
					ROS_ERROR("takeoff_land fail");
				}
			}
			else
			{
				ROS_DEBUG_THROTTLE(3, "Velocity controller: Received control effort, flying the drone");
				// velocity_control.request.frame = frame;
				// velocity_control.request.vx = velocity_control_effort_x;
				// velocity_control.request.vy = velocity_control_effort_y;
				// velocity_control.request.vz = 0;
				// velocity_control.request.yawRate = velocity_control_effort_yaw;

				droneTaskControl.request.task = 4;

				sensor_msgs::Joy controlPosYaw;
				controlPosYaw.axes.push_back(velocity_control_effort_x);
				controlPosYaw.axes.push_back(velocity_control_effort_y);
				controlPosYaw.axes.push_back(0);
				controlPosYaw.axes.push_back(velocity_control_effort_yaw);
				ctrlPosYawPub.publish(controlPosYaw);

				if(!(drone_task_service.call(droneTaskControl) && droneTaskControl.response.result))
				{
					ROS_ERROR("takeoff_land fail");
				}
			}
			
		}
		else
		{
			continue;
		}	
	}

	sdk_permission_control.request.control_enable = 0;
	sdk_permission_control_service.call(sdk_permission_control);

}