#ifndef FAST_MOTION_PLANNING_PLANNER_HPP_
#define FAST_MOTION_PLANNING_PLANNER_HPP_

#include <fast_motion_planning/planningProblem.hpp>

namespace fast_motion_planning {

class Planner {
public:
    virtual ~Planner() {}

    // 规划接口
    virtual void plan(PlanningProblem& ppm) = 0;
};
}  // namespace fast_motion_planning

#endif
