#ifndef ACTION_PUSH_OBJECT_H
#define ACTION_PUSH_OBJECT_H

#include "continual_planning_executive/actionExecutorActionlib.hpp"
#include "continual_planning_executive/symbolicState.h"
#include "shakey_actionlib/PushAction.h"

namespace shakey_planning_actions
{

    class ActionExecutorPushObject : public ActionExecutorActionlib<shakey_actionlib::PushAction,
                                                    shakey_actionlib::PushGoal,
													shakey_actionlib::PushResult>
    {
        public:
            virtual bool fillGoal(shakey_actionlib::PushGoal & goal,
                    const DurativeAction & a, const SymbolicState & current);

            virtual void updateState(const actionlib::SimpleClientGoalState & actionReturnState,
                    const shakey_actionlib::PushResult & result,
                    const DurativeAction & a, SymbolicState & current);
    };

};

#endif
