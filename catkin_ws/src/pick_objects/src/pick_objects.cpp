#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pickUpX = 6;
double pickUpY = 0;
double dropOffX = 0;
double dropOffY = -7;


int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);
  
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp =  ros::Time::now();

  goal.target_pose.pose.position.x = pickUpX;
  goal.target_pose.pose.position.y = pickUpY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending pick up point");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hurray, the robot reached pick up point");
  else
    ROS_INFO("The base failed to reach pick up point for some reason");

  ros::Duration(5).sleep();

  goal.target_pose.pose.position.x = dropOffX;
  goal.target_pose.pose.position.y = dropOffY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending drop off point");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hurray, the robot reached drop off point");
  else
    ROS_INFO("The base failed to reach drop off point for some reason");

  return(0); 

}
 
