/*
Description: Controller:
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_CONTROLLER_HPP_
#define MOTION_PLANNING_TOTURIAL_CONTROLLER_HPP_

// motion_planning_tutorial
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>

// spline basic
#include <tinysplinecxx.h>

namespace motion_planning_tutorial {

enum class SmoothType : uint8_t { BASIC_SPLINE = 0 };

class Controller {
public:
    Controller(const RobotDescription::SharedPtr& robot_description)
        : robot_description_(robot_description) {}

    // 平滑路径:满足c0,c1,c2,c3连续性
    void smoothPath(ProblemDefinition& pd, double max_spenttime, const SmoothType type);

private:
    RobotDescription::SharedPtr robot_description_{nullptr};
};
}  // namespace motion_planning_tutorial
#endif
