#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

double pickUpX = 6;
double pickUpY = 0;
double dropOffX = 0;
double dropOffY = -7;

bool pickingUp = false;
bool droppingOff = false;
bool pickedUp = false;
bool droppedOff = false;


void odometryCallBack (const nav_msgs::Odometry::ConstPtr& msg)
{
  double robotX = msg->pose.pose.position.x;
  double robotY = msg->pose.pose.position.y;

  double distance = 0.0;
  if(!pickingUp){
    distance = sqrt(pow((robotX - pickUpX), 2) + pow((robotY - pickUpY), 2));
    ROS_INFO("Pick up zone distance = %f", distance);
    if(distance <= 0.5){
      ROS_INFO("In the pick up area");
      pickingUp = true;
    }
  }
  else if(!droppingOff){
    distance = sqrt(pow((robotX - dropOffX), 2) + pow((robotY - dropOffY), 2));
    ROS_INFO("Drop zone distance = %f", distance);
    if(distance <= 0.5){
      ROS_INFO("In the drop off area");
      droppedOff = true;
    }
  }    
}

int main(int argc, char**argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
  ros::Subscriber odom_sub = n.subscribe("amcl_pose",10,odometryCallBack);

  visualization_msgs::Marker marker;
  
  uint32_t shape = visualization_msgs::Marker::CUBE;
    
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cube";
  marker.id = 0;

  marker.type = shape;
  
  marker.action = visualization_msgs::Marker::ADD;
  
  marker.pose.position.x = pickUpX;
  marker.pose.position.y = pickUpY;
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
    if (pickingUp && !pickedUp)
    {
      pickedUp = true;
      marker.action = visualization_msgs::Marker::DELETE;
    }
    if (droppingOff && !droppedOff)
    {
      marker.pose.position.x = dropOffX;
      marker.pose.position.y = dropOffY;
      marker.action = visualization_msgs::Marker::ADD;
    } 
    marker_pub.publish(marker);
    ros::spinOnce();
  }
}
