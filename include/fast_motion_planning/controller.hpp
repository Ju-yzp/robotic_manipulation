#ifndef FAST_MOTION_PLANNING_CONTROLLER_HPP_
#define FAST_MOTION_PLANNING_CONTROLLER_HPP_

// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/collision_detector.hpp>
#include <fast_motion_planning/non_uniform_bspline.hpp>
#include <fast_motion_planning/planningProblem.hpp>

namespace fast_motion_planning {

class Controller {
public:
    Controller(const CollisionDetector::SharedPtr& cd) : cd_(cd) {}

    // 平滑轨迹，同时满足动力学约束
    NonUniformBspline smoothPath(const PlanningProblem& ppm);

    // 通过对轨迹进行采样，检查轨迹是否会与障碍物进行碰撞
    bool checkAllTrajectory(NonUniformBspline& trajectory);

private:
    // 碰撞检测器
    CollisionDetector::SharedPtr cd_;
};
}  // namespace fast_motion_planning
#endif
