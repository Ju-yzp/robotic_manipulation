/*
Description: Ur5eKinameticSolver: a class for describing ur5e robot kinematic model
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef FAST_MOTION_PLANNING_KINEMATIC_INTERFACE_HPP_
#define FAST_MOTION_PLANNING_KINEMATIC_INTERFACE_HPP_

// eigen
#include <Eigen/Eigen>

// fast_motion_planning
#include <fast_motion_planning/types.hpp>

// cpp
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace fast_motion_planning {

class KinematicInterface {
public:
    REGISTER_SMART_POINTER(KinematicInterface);

    KinematicInterface(const int dof) : dof_(dof) {}

    virtual ~KinematicInterface() {}

    // 逆运动学接口
    virtual std::vector<State> inverseKinematic(const Eigen::Matrix4d end_effector_pose) = 0;

    // 正运动学接口
    virtual std::unordered_map<std::size_t, Eigen::Matrix4d> forwardKinematic(
        const State state) = 0;

protected:
    // 机械臂轴数
    int dof_{0};
};
}  // namespace fast_motion_planning

#endif
