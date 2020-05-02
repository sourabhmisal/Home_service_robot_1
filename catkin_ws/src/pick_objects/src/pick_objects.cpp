#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_msgs/Int32.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double pickUpX = 6;
double pickUpY = 0;
double dropOffX = 0;
double dropOffY = -6;

enum action {goingToPickUp = 1, pickedUp = 2, goingToDropOff = 3, droppedOff = 4};

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  MoveBaseClient ac("move_base", true);
  
  ros::Publisher status_pub = n.advertise<std_msgs::Int32>("robot_status", 1);
  
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::Int32 statusMsg;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp =  ros::Time::now();

  goal.target_pose.pose.position.x = pickUpX;
  goal.target_pose.pose.position.y = pickUpY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending pick up point");
  statusMsg.data = goingToPickUp;
  status_pub.publish(statusMsg);
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {  
    ROS_INFO("Hurray, the robot reached pick up point");
    statusMsg.data = pickedUp;
    status_pub.publish(statusMsg);
  }
  else
    ROS_INFO("The base failed to reach pick up point for some reason");

  sleep(5);

  goal.target_pose.pose.position.x = dropOffX;
  goal.target_pose.pose.position.y = dropOffY;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending drop off point");
  statusMsg.data = goingToDropOff;
  status_pub.publish(statusMsg);
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {  ROS_INFO("Hurray, the robot reached drop off point");
     statusMsg.data = droppedOff;
     status_pub.publish(statusMsg);
  }
  else
    ROS_INFO("The base failed to reach drop off point for some reason");
  
  sleep(5);
  return(0); 

}
