#ifndef FAST_MOTION_PLANNING_CONTROLLER_HPP_
#define FAST_MOTION_PLANNING_CONTROLLER_HPP_

#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/non_uniform_bspline.hpp>
#include <fast_motion_planning/planningProblem.hpp>

namespace fast_motion_planning {

class Controller {
public:
    Controller(RobotParams::SharedPtr robot_params) : robot_params_(robot_params) {}

    NonUniformBspline smoothPath(const PlanningProblem& ppm);

private:
    RobotParams::SharedPtr robot_params_;
};
}  // namespace fast_motion_planning
#endif
