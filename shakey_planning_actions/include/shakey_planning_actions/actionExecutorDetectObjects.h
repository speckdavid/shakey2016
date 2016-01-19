#ifndef ACTION_EXECUTOR_DETECT_OBJECTS_H
#define ACTION_EXECUTOR_DETECT_OBJECTS_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"

namespace tidyup_actions
{

    class ActionExecutorDetectObjects : public ActionExecutorService<shakey_actionlib::ObserveAction>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(shakey_actionlib::ObserveAction & goal,
                    const DurativeAction & a, const SymbolicState & current);

//            virtual bool executeBlocking(const DurativeAction & a, SymbolicState & current);

            virtual void updateState(bool & success, tidyup_msgs::DetectGraspableObjects::Response & response,
                    const DurativeAction & a, SymbolicState & current);

        private:
            ros::ServiceClient serviceClientGraspability;
            bool requestGraspability;
            string tidyLocationName;

            std::string findStaticObjectForLocation(const std::string& location, const SymbolicState & current) const;
//            actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction>* headControl;
//            string head_pointing_frame;
    };

};

#endif
