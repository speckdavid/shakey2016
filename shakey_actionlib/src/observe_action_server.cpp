#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <shakey_actionlib/ObserveAction.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

class ObserveAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<shakey_actionlib::ObserveAction> as_;
  std::string action_name_;
  tf::TransformListener listener_;

public:

  ObserveAction(std::string name) :
    as_(nh_, name, boost::bind(&ObserveAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~ObserveAction(void)
  {
  }

  void executeCB(const shakey_actionlib::ObserveGoalConstPtr &goal)
  {
	// In Threath?
	/*system("roslaunch shakey_object_recognition shakey_object_recognition.launch");
    as_.setPreempted();
	// ros::Duration(10).sleep();
    ROS_INFO("Observe action completed.");
    // system("rosnode kill /segmentation3d ");*/
    as_.setSucceeded();
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "observe_action_server");

  ObserveAction push(ros::this_node::getName());
  ros::spin();

  return 0;
}
