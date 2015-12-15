#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <shakey_actionlib/PushAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "shakey_push_client");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<shakey_actionlib::PushAction> ac("shakey_push_server", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  shakey_actionlib::PushGoal goal;
  goal.target_point.x = -3.2;
  goal.target_point.y = 3.25;
  goal.target_point.z = 0;
  goal.push_distance = 1.5;
  goal.push_direction.x = -0.47;
  goal.push_direction.y = -0.15;
  goal.push_direction.z = 0;
  ac.sendGoal(goal);

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
