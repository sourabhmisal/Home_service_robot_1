#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

double pickUpPoint[3] = {6,0,-1.59};
double dropOffPoint[3] = {0,-7,1};

int main(int argc, char**argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  int action = visualization_msgs::Marker::ADD;

  bool pickUpPosition = false;
  bool dropOffPosition = false;

  while(ros::ok()){
    visualization_msgs::Marker marker;
    
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = shape;
    
    marker.pose.position.x = pickUpPoint[0];
    marker.pose.position.y = pickUpPoint[1];
    marker.pose.position.z = 0.05;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 0;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    while(marker_pub.getNumSubscribers() < 1) 
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN("Please create a subscriber to the marker");
      sleep(1);
    }  

    if (!pickUpPosition)
    {
      marker.action = visualization_msgs::Marker::ADD;
      pickUpPosition = true;
      marker_pub.publish(marker);
      sleep(5);
    }
    
    else if (!dropOffPosition)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      dropOffPosition = true;
      marker_pub.publish(marker);
      sleep(5);
    } 
    else 
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = dropOffPoint[0];
      marker.pose.position.y = dropOffPoint[1];
      marker_pub.publish(marker);
    }



    ros::spinOnce();
  }
}
