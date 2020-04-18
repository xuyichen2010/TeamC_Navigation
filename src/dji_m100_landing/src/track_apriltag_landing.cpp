#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <ros/console.h>

#include "apriltag_ros/AprilTagDetection.h"
#include "apriltag_ros/AprilTagDetectionArray.h"

#include <tag.h>

#include <cmath>
#include <Eigen/Geometry>

#include <vector>
#include <iterator>
#include <algorithm>

#include <sensor_msgs/Joy.h>

#include "dji_sdk/dji_sdk.h"
#include <dji_sdk/DroneTaskControl.h>


ros::Subscriber apriltags_36h11_sub;
ros::Subscriber local_position_sub;
ros::Subscriber landing_enable_sub;
ros::Subscriber attitude_quaternion_sub;
ros::Subscriber flight_status_sub;
ros::Subscriber global_position_sub;

ros::Publisher ctrlPosYawPub;

ros::ServiceClient drone_task_service;

double local_x;
double local_y;
double local_z;
double flight_height;

double heading_q0;
double heading_q1;
double heading_q2;
double heading_q3;

Eigen::Quaternion<double> drone_heading;
double yaw_state = 0;

double setpoint_x = 0;
double setpoint_y = 0;
double setpoint_z = 0;
double setpoint_yaw = 0;

double landing_height_threshold = 1;
double landing_center_threshold = 0.5;

int flight_status;

Tag *tag_36h11_0;

double yaw_error;

std::string tag_36h11_detection_topic;

bool found_36h11 = false;
bool landing_enabled = false;
bool during_landing = false;
bool continue_landing = false;


void apriltags36h11Callback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& apriltag_pos_msg)
{
  if(std::begin(apriltag_pos_msg->detections) == std::end(apriltag_pos_msg->detections))
  {
    found_36h11 = false;
    return;
  }
  else
  {
    for(auto it = std::begin(apriltag_pos_msg->detections); it != std::end(apriltag_pos_msg->detections); ++ it)
    {
      if((*it).id[0] == 0)
      {
        tag_36h11_0->updateTagState((*it).pose.pose.pose);
        found_36h11 = true;
      }
    }
  }
}

void localPositionCallback(const geometry_msgs::PointStamped::ConstPtr& local_position_msg)
{
  local_x = local_position_msg->point.x;
  local_y = local_position_msg->point.y;
  local_z = local_position_msg->point.z;
}

void attitudeQuaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& attitude_quaternion_msg)
{
  heading_q0 = attitude_quaternion_msg->quaternion.w;
  heading_q1 = attitude_quaternion_msg->quaternion.x;
  heading_q2 = attitude_quaternion_msg->quaternion.y;
  heading_q3 = attitude_quaternion_msg->quaternion.z;

  drone_heading = Eigen::Quaternion<double>(heading_q0, heading_q1, heading_q2, heading_q3);
  yaw_state = atan2(2*(heading_q0 * heading_q3 + heading_q1 * heading_q2), 1 - 2 * (heading_q2 * heading_q2 + heading_q3 * heading_q3)) / M_PI * 180;
}

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& global_position_msg)
{
  flight_height = global_position_msg->altitude;
}

void landingEnableCallback(const std_msgs::Bool& landing_enable_msg)
{
  landing_enabled = landing_enable_msg.data;
}

void flightStatusCallback(const std_msgs::UInt8& flight_status_msg)
{
  flight_status = (int)flight_status_msg.data;
}

void print_parameters()
{
  ROS_INFO("Listening to 36h11 apriltag detection topic: %s", tag_36h11_detection_topic.c_str());
  ROS_INFO("landing_height_threshold: %f", landing_height_threshold);
  ROS_INFO("landing_center_threshold: %f", landing_center_threshold);
}

bool land()
{
  ROS_INFO("land().");
  dji_sdk::DroneTaskControl droneTaskControl;
  droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
  drone_task_service.call(droneTaskControl);
  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("landing fail");
    return false;
  }
  ROS_INFO("Return TRue.");
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "track_apriltag_landing_node");
  ROS_INFO("Starting apriltag track and landing publisher");
  ros::NodeHandle nh;
  ros::NodeHandle node_priv("~");

  node_priv.param<std::string>("tag_36h11_detection_topic", tag_36h11_detection_topic, "/tag_detections");
  node_priv.param<double>("landing_height_threshold", landing_height_threshold, 1);
  node_priv.param<double>("landing_center_threshold", landing_center_threshold, 0.5);

  print_parameters();

  apriltags_36h11_sub = nh.subscribe(tag_36h11_detection_topic, 1, apriltags36h11Callback);
  local_position_sub = nh.subscribe("dji_sdk/local_position", 10, localPositionCallback);
  attitude_quaternion_sub = nh.subscribe("/dji_sdk/attitude", 1, attitudeQuaternionCallback );
  flight_status_sub = nh.subscribe("/dji_sdk/flight_status", 1, flightStatusCallback);
  global_position_sub = nh.subscribe("/dji_sdk/gps_position", 10, globalPositionCallback);

  landing_enable_sub = nh.subscribe("/dji_landing/landing_enable", 1, landingEnableCallback );

  ctrlPosYawPub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  drone_task_service = nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");

  tag_36h11_0 = new Tag();
  // Set the translation between camera and landing center
  tag_36h11_0->setToLandingCenterTranslation(Eigen::Vector3d(0.0, 0.0, 0.0));

  //camera to drone transformation
  Eigen::Matrix3d camera_to_drone_transformation;
  camera_to_drone_transformation << 0, -1, 0,
  -1, 0, 0,
  0, 0, -1;

  Eigen::Vector3d landing_center_position;

  ros::Rate loop_rate(10);

  ros::spinOnce();

  while (ros::ok())
  {
    ros::spinOnce();

    //ROS_INFO("flight_status:%d \n", flight_status);
    //ROS_INFO("suppose:%d \n", DJISDK::M100FlightStatus::M100_STATUS_IN_AIR);

    // Check if the UAV is currently in air, if not in air, then UAV cannot land
    if(flight_status != DJISDK::M100FlightStatus::M100_STATUS_IN_AIR)
    {
      ROS_ERROR("The UAV is not in air, landing cannot start!");
      continue;
    }

    // Check if received command for landing
    if(landing_enabled)
    {
      ROS_INFO_ONCE("Landing is enabled.");
      // Found apriltag, start landing
      if(found_36h11)
      {
        ROS_INFO_ONCE("Found Apriltag, start landing.");

        // get the landing center position refer to the local frame
        tag_36h11_0->calculateDroneFramePosition(camera_to_drone_transformation);
        tag_36h11_0->calculateDroneFrameOrientation(camera_to_drone_transformation);
        landing_center_position = tag_36h11_0->getLandingCenterPosition();

        // get the landing center yaw error
        yaw_error = (tag_36h11_0->getYawError())/ M_PI * 180;

        // Convert the x,y position from local frame to ground NEU frame
        double yaw_angle_radian = (yaw_state/180)* M_PI;
        double delta_x = landing_center_position(0)*cos(yaw_angle_radian) - landing_center_position(1)*sin(yaw_angle_radian);
        double delta_y = landing_center_position(0)*sin(yaw_angle_radian) + landing_center_position(1)*cos(yaw_angle_radian);

        // Set the Joy message and publish to flight_control_setpoint_ENUposition_yaw topic
        setpoint_x = delta_x;
        setpoint_y = delta_y;
        setpoint_yaw = yaw_state + yaw_error;

        // Move above the tag first, then land
        if(sqrt(pow(delta_x, 2) + pow(delta_y, 2)) > landing_center_threshold)
        {
          setpoint_z = flight_height;
          ROS_INFO("Not Land: ");
          ROS_INFO_STREAM(sqrt(pow(delta_x, 2) + pow(delta_y, 2)));
        }
        else{
          // setpoint_z = landing_center_position(2) + local_z;
          ROS_INFO("landing_center_position(2)");
          ROS_INFO_STREAM(landing_center_position(2));
          ROS_INFO("local_z.");
          ROS_INFO_STREAM(local_z);
          setpoint_z = landing_center_position(2);
        }

        sensor_msgs::Joy controlPosYaw;
        controlPosYaw.axes.push_back(setpoint_x);
        controlPosYaw.axes.push_back(setpoint_y);
        controlPosYaw.axes.push_back(setpoint_z);
        controlPosYaw.axes.push_back(setpoint_yaw);
        ctrlPosYawPub.publish(controlPosYaw);

        ROS_INFO("The x offset, y offset, z, yaw in ground frame of apriltag.");
        ROS_INFO_STREAM(setpoint_x);
        ROS_INFO_STREAM(setpoint_y);
        ROS_INFO_STREAM(setpoint_z);
        ROS_INFO_STREAM(setpoint_yaw);

        during_landing = true;
        continue_landing = true;
      }
      else
      {
        if (during_landing)
        {
          ROS_INFO_ONCE("Lost apriltag during landing!");
          ROS_INFO_STREAM(local_z);
          // If height is very low, start dji landing
          if (local_z <= landing_height_threshold)
          {
            if (land())
            {
              ROS_INFO_ONCE("Continue landing.");
              if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_ON_GROUND)
              {
                ROS_INFO_ONCE("Successful landing!");
                during_landing = false;
                continue_landing = false;
              }
            }
          }
          // continue to fly towards the last target pose, only once
          else
          {
            if (continue_landing)
            {
              sensor_msgs::Joy controlPosYaw;
              controlPosYaw.axes.push_back(setpoint_x);
              controlPosYaw.axes.push_back(setpoint_y);
              controlPosYaw.axes.push_back(setpoint_z);
              controlPosYaw.axes.push_back(setpoint_yaw);
              ctrlPosYawPub.publish(controlPosYaw);
            }
            during_landing = false;
            continue_landing = false;
          }
        }
        else
        {
          ROS_ERROR_ONCE("No apriltag found, landing suspend!");
        }
      }

      if(flight_status == DJISDK::M100FlightStatus::M100_STATUS_FINISHED_LANDING){
        ROS_INFO_ONCE("Successful landing!");
      }

      loop_rate.sleep();
    }
  }
}
