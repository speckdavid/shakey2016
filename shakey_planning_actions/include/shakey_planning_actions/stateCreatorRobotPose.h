#ifndef STATE_CREATOR_ROBOT_POSE_H
#define STATE_CREATOR_ROBOT_POSE_H

#include "continual_planning_executive/stateCreator.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace shakey_planning_actions
{
    class StateCreatorRobotPose : public continual_planning_executive::StateCreator
    {
        public:
            StateCreatorRobotPose();
            ~StateCreatorRobotPose();

            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillState(SymbolicState & state);
        protected:
            /// Extract a PoseStamped for object from state.
            /**
            * The fluents that are queried are: x,y,z, qx,qy,qz,qw, frame-id, timestamp
            *
            * \returns true if all fluents were available
            */
            bool extractPoseStamped(const SymbolicState & state, const string & object,
            		geometry_msgs::PoseStamped & pose) const;

        protected:
            tf::TransformListener _tf;

            double _goalToleranceXY;
            double _goalToleranceYaw;

            std::string _robotPoseObject;   ///< the name of the robot pose's object (e.g. robot_pose, or l0)
            std::string _robotPoseType;     ///< the type of the _robotPoseObject - required if _robotPoseObject.
            std::string _atPredicate;       ///< the name of the "at" predicate (e.g. at-base)
            std::string _locationType;      ///< the type of location objects that a robot might be "at"
    };

};

#endif
