#ifndef GOAL_CREATOR_OBJECTS_H
#define GOAL_CREATOR_OBJECTS_H

#include "continual_planning_executive/goalCreator.h"

namespace shakey_planning_actions
{

    class GoalCreatorObjects : public continual_planning_executive::GoalCreator
    {
    private:
        public:
            GoalCreatorObjects();
            ~GoalCreatorObjects();

            virtual bool fillStateAndGoal(SymbolicState & currentState, SymbolicState & goal);
    };

};

#endif
