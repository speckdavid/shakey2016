#include "shakey_planning_actions/stateCreatorRobotPose.h"
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>


PLUGINLIB_DECLARE_CLASS(shakey_planning_actions, state_creator_robot_pose,
        shakey_planning_actions::StateCreatorRobotPose, continual_planning_executive::StateCreator)

namespace shakey_planning_actions
{

    StateCreatorRobotPose::StateCreatorRobotPose()
    {
    	ros::NodeHandle nhPriv("~");
    	ros::NodeHandle nh;
    	nhPriv.param("nav_target_tolerance_xy", _goalToleranceXY, 0.5);
    	nhPriv.param("nav_target_tolerance_yaw", _goalToleranceYaw, 0.26);  //15deg

    	bool relative;
    	nhPriv.param("nav_target_tolerance_relative_to_move_base", relative, false);
    	if(relative) {
    		// relative mode: 1. get the namespace for base_local_planner
    	    std::string base_local_planner_ns;
    	    if(!nhPriv.getParam("nav_base_local_planner_ns", base_local_planner_ns)) {
    	    	ROS_WARN("nav_target_tolerance_relative_to_move_base set, but nav_base_local_planner_ns not set - trying to estimate");
    	        std::string local_planner;
    	        if(!nh.getParam("move_base_node/base_local_planner", local_planner)
    	        		&& !nh.getParam("move_base/base_local_planner", local_planner)) {
    	        	ROS_ERROR("move_base(_node)/base_local_planner not set - falling back to absolute mode.");
    	        } else {
    	        	// dwa_local_planner/DWAPlannerROS -> DWAPlannerROS
    	            std::string::size_type x = local_planner.find_last_of("/");
    	            if(x == std::string::npos)
    	            	base_local_planner_ns = local_planner;
    	            else {
    	            	base_local_planner_ns = local_planner.substr(x + 1);
    	                ROS_INFO("Estimated base_local_planner_ns to %s.", base_local_planner_ns.c_str());
    	                }
    	        }

    	        if(!base_local_planner_ns.empty()) { // success: 2. get the xy_goal_tolerance
    	        	double move_base_tol_xy;
    	            if(!nh.getParam(base_local_planner_ns + "/xy_goal_tolerance", move_base_tol_xy)) {
    	                ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
    	                        << (base_local_planner_ns + "/xy_goal_tolerance") << " was not set"
    	                        << " - falling back to absolute mode");
    	            } else { // 2. add move_base's tolerance to our relative tolerance
    	            	_goalToleranceXY += move_base_tol_xy;
    	            }

    	            double move_base_tol_yaw;
    	            if(!nh.getParam(base_local_planner_ns + "/yaw_goal_tolerance", move_base_tol_yaw)) {
    	               ROS_ERROR_STREAM("nav_target_tolerance_relative_to_move_base was true, but "
    	                       << (base_local_planner_ns + "/yaw_goal_tolerance") << " was not set"
    	                       << " - falling back to absolute mode");
    	            } else { // 2. add move_base's tolerance to our relative tolerance
    	            	_goalToleranceYaw += move_base_tol_yaw;
    	            }
    	        }
    	    }
    	}
    }

    StateCreatorRobotPose::~StateCreatorRobotPose()
    {
    }

    void StateCreatorRobotPose::initialize(const std::deque<std::string> & arguments)
    {
    	ROS_ASSERT(arguments.size() == 4);

    	_robotPoseObject = arguments[0];
    	_robotPoseType = arguments[1];
    	_atPredicate = arguments[2];
    	_locationType = arguments[3];

    	if(_robotPoseObject == "-")
    		_robotPoseObject = "";
    	if(_robotPoseType == "-")
    		_robotPoseType = "";
    	if(_atPredicate == "-")
    		_atPredicate = "";
    	if(_locationType == "-")
    		_locationType = "";
    }

    bool StateCreatorRobotPose::fillState(SymbolicState & state)
    {
    	// For test
    	state.addObject("loc1", _robotPoseType);
    	state.addObject(_robotPoseObject, _robotPoseType);
    	state.setNumericalFluent("x", "loc1", 3);
    	state.setNumericalFluent("y", "loc1", 3);
    	state.setNumericalFluent("z", "loc1", 0);
    	state.setNumericalFluent("qx", "loc1", 0);
    	state.setNumericalFluent("qy", "loc1", 0);
    	state.setNumericalFluent("qz", "loc1", 0);
    	state.setNumericalFluent("qw", "loc1", 1);
    	state.setNumericalFluent("timestamp", "loc1", ros::Time::now().toSec());
    	state.setObjectFluent("frame-id", "loc1", "/map");

        tf::StampedTransform transform;
        try{
        	_tf.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
            _tf.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException& ex){
            ROS_ERROR("%s",ex.what());
            return false;
        }

        // 1. Real robot location
        if(!_robotPoseObject.empty()) {
            ROS_ASSERT(!_robotPoseType.empty());
            state.addObject(_robotPoseObject, _robotPoseType);
            state.setNumericalFluent("x", _robotPoseObject, transform.getOrigin().x());
            state.setNumericalFluent("y", _robotPoseObject, transform.getOrigin().y());
            state.setNumericalFluent("z", _robotPoseObject, transform.getOrigin().z());
            state.setNumericalFluent("qx", _robotPoseObject, transform.getRotation().x());
            state.setNumericalFluent("qy", _robotPoseObject, transform.getRotation().y());
            state.setNumericalFluent("qz", _robotPoseObject, transform.getRotation().z());
            state.setNumericalFluent("qw", _robotPoseObject, transform.getRotation().w());
            state.setNumericalFluent("timestamp", _robotPoseObject, ros::Time::now().toSec());
            state.addObject("/map", "frameid");
            state.setObjectFluent("frame-id", _robotPoseObject, "/map");
        }

        // 2.b check if we are at any _locations
        pair<SymbolicState::TypedObjectConstIterator, SymbolicState::TypedObjectConstIterator> targets =
            state.getTypedObjects().equal_range(_locationType);

        double minDist = HUGE_VAL;
        string nearestTarget = "";

        int atLocations = 0;
        for(SymbolicState::TypedObjectConstIterator it = targets.first; it != targets.second; it++) {
            string target = it->second;
            if(target == _robotPoseObject)  // skip current robot location
                continue;

            geometry_msgs::PoseStamped targetPose;
            if(!extractPoseStamped(state, target, targetPose)) {
                ROS_ERROR("%s: could not extract pose for target object: %s.", __func__, target.c_str());
                continue;
            }
            if(targetPose.header.frame_id != "/map") {
                ROS_ERROR("Target pose %s had frame-id: %s - should be /map.",
                        target.c_str(), targetPose.header.frame_id.c_str());
                continue;
            }

            // compute dXY, dYaw between current pose and target
            tf::Transform targetTransform;//(btQuaternion(qx, qy, qz, qw), btVector3(posX, posY, 0.0));
            tf::poseMsgToTF(targetPose.pose, targetTransform);
            tf::Transform deltaTransform = targetTransform.inverseTimes(transform);

            double dDist = hypot(deltaTransform.getOrigin().x(), deltaTransform.getOrigin().y());
            double dAng = tf::getYaw(deltaTransform.getRotation());
            ROS_INFO("Target %s dist: %f m ang: %f deg", target.c_str(), dDist, angles::to_degrees(dAng));

            if(!_atPredicate.empty()) {
                // Found a target - update state!
                if(dDist < _goalToleranceXY && fabs(dAng) < _goalToleranceYaw) {
                    ROS_INFO("(at) target %s !", target.c_str());
                    state.setBooleanPredicate(_atPredicate, target, true);
                    atLocations++;
                } else {
                    state.setBooleanPredicate(_atPredicate, target, false);
                }
                if(dDist < minDist) {
                    minDist = dDist;
                    nearestTarget = target;
                }
            }
        }

        ROS_INFO("Nearest target is %s (%f m).", nearestTarget.c_str(), minDist);

        // 2.a Set the robot pose, if we are not already at another pose
        if(!_atPredicate.empty() && !_robotPoseObject.empty()) {
            if(atLocations == 0) {
                state.setBooleanPredicate(_atPredicate, _robotPoseObject, true);
            } else {
                state.setBooleanPredicate(_atPredicate, _robotPoseObject, false);
                if(atLocations > 1) {
                    ROS_WARN("We are at %d locations at the same time!.", atLocations);
                }
            }
        }

        return true;
    }

    bool StateCreatorRobotPose::extractPoseStamped(const SymbolicState & state, const string & object,
                geometry_msgs::PoseStamped & pose) const
        {
            bool ret = true;

            // first get xyz, qxyzw from state
            Predicate p;
            p.parameters.push_back(object);

            double posX = 0;
            p.name = "x";
            if(!state.hasNumericalFluent(p, &posX)) {
                ROS_ERROR("%s: object: %s - no x-location in state.", __func__, object.c_str());
                ret = false;;
            }
            double posY = 0;
            p.name = "y";
            if(!state.hasNumericalFluent(p, &posY)) {
                ROS_ERROR("%s: object: %s - no y-location in state.", __func__, object.c_str());
                ret = false;;
            }
            double posZ = 0;
            p.name = "z";
            if(!state.hasNumericalFluent(p, &posZ)) {
                ROS_ERROR("%s: object: %s - no z-location in state.", __func__, object.c_str());
                ret = false;;
            }

            double qx;
            p.name = "qx";
            if(!state.hasNumericalFluent(p, &qx)) {
                ROS_ERROR("%s: object: %s - no qx in state.", __func__, object.c_str());
                ret = false;;
            }
            double qy;
            p.name = "qy";
            if(!state.hasNumericalFluent(p, &qy)) {
                ROS_ERROR("%s: object: %s - no qy in state.", __func__, object.c_str());
                ret = false;;
            }
            double qz;
            p.name = "qz";
            if(!state.hasNumericalFluent(p, &qz)) {
                ROS_ERROR("%s: object: %s - no qz in state.", __func__, object.c_str());
                ret = false;;
            }
            double qw;
            p.name = "qw";
            if(!state.hasNumericalFluent(p, &qw)) {
                ROS_ERROR("%s: object: %s - no qw in state.", __func__, object.c_str());
                ret = false;;
            }

            double timestamp;
            p.name = "timestamp";
            if(!state.hasNumericalFluent(p, &timestamp)) {
                ROS_ERROR("%s: object: %s - no timestamp in state.", __func__, object.c_str());
                ret = false;;
            }

            string frameid;
            p.name = "frame-id";
            if(!state.hasObjectFluent(p, &frameid)) {
                ROS_ERROR("%s: object: %s - no frameid in state.", __func__, object.c_str());
                ret = false;;
            }

            pose.header.frame_id = frameid;
            pose.header.stamp = ros::Time(timestamp);
            pose.pose.position.x = posX;
            pose.pose.position.y = posY;
            pose.pose.position.z = posZ;
            pose.pose.orientation.x = qx;
            pose.pose.orientation.y = qy;
            pose.pose.orientation.z = qz;
            pose.pose.orientation.w = qw;

            return ret;
        }

};
