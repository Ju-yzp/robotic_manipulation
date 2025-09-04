/*
Description: KinematicBaseInterface: a pure virtual base class for kinematic interfaces
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_KINEMATIC_BASE_INTERFACE_HPP_
#define MOTION_PLANNING_TOTURIAL_KINEMATIC_BASE_INTERFACE_HPP_

// eigen
#include <Eigen/Eigen>

// cpp
#include <memory>
#include <unordered_map>

// motion_planning_tutorial
#include <motion_planning_tutorial/state.hpp>

namespace motion_planning_tutorial {
class KinematicBaseInterface {  // 运动学纯虚基类接口
public:
    using SharedPtr = std::shared_ptr<KinematicBaseInterface>;
    using UniquePtr = std::unique_ptr<KinematicBaseInterface>;

    KinematicBaseInterface() = default;

    virtual ~KinematicBaseInterface() = default;

    virtual std::vector<State> inverseKinematic(
        const Eigen::Isometry3d& goal_pose) = 0;  // 逆运动学接口

    virtual std::unordered_map<std::string, Eigen::Isometry3d> forwardKinematic(
        const State& state) = 0;  // 正运动学接口
};
}  // namespace motion_planning_tutorial

#endif
