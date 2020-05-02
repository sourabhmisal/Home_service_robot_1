#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double pickUpX = 6;
double pickUpY = 0;
double dropOffX = 0;
double dropOffY = -6;

enum action {goingToPickUp = 1, pickedUp = 2, goingToDropOff = 3, droppedOff = 4};
int state = 0;

void statusChange(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("Status Changed!");
  switch(msg->data)
  {
    case goingToPickUp:
      ROS_INFO("Status: On way to pick up");
      break;
    case pickedUp:
      ROS_INFO("Status: Picked up item");
      break;
    case goingToDropOff:
      ROS_INFO("Status: On way to drop off");
      break;
    case droppedOff:
      ROS_INFO("Status: Dropped off item");
      break;
  } 
  state = msg->data;
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
  ros::Subscriber status_sub = n.subscribe("robot_status",1,statusChange);

  visualization_msgs::Marker marker;
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
    
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cube";
  marker.id = 0;

  marker.type = shape;
  
  marker.pose.position.z = 0.05;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while(ros::ok()){

    while(marker_pub.getNumSubscribers() < 1) 
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }

    switch(state)
    {
      case goingToPickUp:
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pickUpX;
        marker.pose.position.y = pickUpY;
        break;
      case pickedUp:
        marker.action = visualization_msgs::Marker::DELETE;
        break;   
      case goingToDropOff:
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      case droppedOff:
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = dropOffX;
        marker.pose.position.y = dropOffY;
        break;
    }  

    marker_pub.publish(marker);
    ros::spinOnce();
  }
}
