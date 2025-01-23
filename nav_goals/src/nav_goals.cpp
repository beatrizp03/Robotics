#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  if(argc != 4)
  {
      std::cout << "Usage: ./nav_goals <goal_x> <goal_y> <goal_w>\n";
      std::cout << "\tgoal_x: x coord (in the map frame) of the goal\n\tgoal_y: y coord (in the map frame) of the goal\n\tgoal_w: yaw coord (in the map frame) of the goal\n";
  
      return -1;
  }
  
  ros::init(argc, argv, "nav_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = std::stod(argv[1]);
  goal.target_pose.pose.position.y = std::stod(argv[2]);

  tf2::Quaternion q;
  q.setRPY(0, 0, std::stod(argv[3]));
  geometry_msgs::Quaternion quaternion;
  quaternion = tf2::toMsg(q);
  goal.target_pose.pose.orientation = quaternion;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("The goal was not reached for some reason.");
    
  // Send other position -------------------------------------------------------
  
  /*goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = -1.75;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending another goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Goal reached!");
  else
    ROS_INFO("The goal was not reached for some reason.");*/

  return 0;
}
