#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pickUpPoint[3] = {1,0,-1.59};
double dropOffPoint[3] = {0,1,1};

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");

  MoveBaseClient ac("move_base", true);
  
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp =  ros::Time::now();

  goal.target_pose.pose.position.x = pickUpPoint[0];
  goal.target_pose.pose.position.y = pickUpPoint[1];
  goal.target_pose.pose.orientation.w = pickUpPoint[2];

  ROS_INFO("Sending pick up point");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hurray, the robot reached pick up point");
  else
    ROS_INFO("The base failed to reach pick up point for some reason");

  ros::Duration(5).sleep();

  goal.target_pose.pose.position.x = dropOffPoint[0];
  goal.target_pose.pose.position.y = dropOffPoint[1];
  goal.target_pose.pose.orientation.w = dropOffPoint[2];

  ROS_INFO("Sending drop off point");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hurray, the robot reached drop off point");
  else
    ROS_INFO("The base failed to reach drop off point for some reason");

  return(0); 

}
 
