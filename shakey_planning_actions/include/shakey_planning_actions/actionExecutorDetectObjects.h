#ifndef ACTION_EXECUTOR_DETECT_OBJECTS_H
#define ACTION_EXECUTOR_DETECT_OBJECTS_H

#include "continual_planning_executive/actionExecutorService.hpp"
#include "continual_planning_executive/symbolicState.h"
#include <actionlib/client/simple_action_client.h>
#include <shakey_object_recognition/DetectObjects.h>
#include "shakey_object_recognition/ObjectVisualisation.h"
#include <nav_msgs/GetMap.h>

namespace shakey_planning_actions
{

    class ActionExecutorDetectObjects : public ActionExecutorService<shakey_object_recognition::DetectObjects>
    {
        public:
            virtual void initialize(const std::deque<std::string> & arguments);

            virtual bool fillGoal(shakey_object_recognition::DetectObjects::Request & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(bool & success, shakey_object_recognition::DetectObjects::Response & response,
                    const DurativeAction & a, SymbolicState & current);

        protected:
            void getBestPushPose(shakey_object_recognition::PushableObject);

            ObjectVisualisation objVis;
    };

};

#endif
