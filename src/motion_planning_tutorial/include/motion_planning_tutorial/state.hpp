#ifndef MOTION_PLANNING_TOTURIAL_STATE_HPP_
#define MOTION_PLANNING_TOTURIAL_STATE_HPP_

#include <Eigen/Eigen>

namespace motion_planning_tutorial {

struct State {
    Eigen::VectorXd positions;
};
}  // namespace motion_planning_tutorial

#endif
