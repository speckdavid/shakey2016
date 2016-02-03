#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <shakey_actionlib/PushAction.h>
#include <shakey_actionlib/MoveAction.h>
#include <shakey_actionlib/ObserveAction.h>
#include <tf/transform_listener.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "shakey_action_client");

  /*actionlib::SimpleActionClient<shakey_actionlib::ObserveAction> ac1("observe_action_server", true);
  ROS_INFO("Waiting for action server to start.");
  ac1.waitForServer();
  ROS_INFO("Send goal.");
  shakey_actionlib::ObserveGoal goal;
  ac1.sendGoalAndWait(goal);*/

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<shakey_actionlib::MoveAction> ac1("move_action_server", true);
  ROS_INFO("Waiting for action server to start.");
  ac1.waitForServer();
  shakey_actionlib::MoveGoal goal1;
  goal1.target_pose.position.x = 0.975404;
  goal1.target_pose.position.y = 6.5046;
  goal1.target_pose.position.z = 0.0;
  // Set correct orientation
  tf::Vector3 plane_norm(0.312877, -0.38553, 0.0);
  plane_norm = plane_norm.normalized();
  std::cerr << plane_norm.getX() << plane_norm.getY() << plane_norm.getZ() << std::endl;
  tf::Quaternion qt = tf::shortestArcQuat (tf::Vector3(1,0,0), plane_norm);
  geometry_msgs::Quaternion robot_orientation;
  tf::quaternionTFToMsg(qt, robot_orientation);
  std::cerr << robot_orientation << std::endl;
  goal1.target_pose.orientation = robot_orientation;
  ac1.sendGoalAndWait(goal1);

  actionlib::SimpleActionClient<shakey_actionlib::PushAction> ac2("push_action_server", true);
  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac2.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  shakey_actionlib::PushGoal goal2;
  ac2.sendGoalAndWait(goal2);

  //wait for the action to return
  /*bool finished_before_timeout = ac.waitForResult(ros::Duration(120.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");*/

  //exit
  return 0;
}
