/*
Description: Ur5eKinameticSolver: a class for describing ur5e robot kinematic model
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_UR5E_KINEMATIC_HPP_
#define MOTION_PLANNING_TOTURIAL_UR5E_KINEMATIC_HPP_

// cpp
#include <array>
#include <cstddef>

// motion_planning_tutorial
#include <fast_motion_planning/kinematic_interface.hpp>
#include <fast_motion_planning/types.hpp>

namespace fast_motion_planning {

using Ur5eParam = std::array<double, 6>;

class Ur5eKinematicSolver : virtual public KinematicInterface {
public:
    REGISTER_SMART_POINTER(Ur5eKinematicSolver);

    Ur5eKinematicSolver(
        const Ur5eParam at, const Ur5eParam dt, const Ur5eParam apt, const Ur5eParam tht)
        : a_table_(at),
          d_table_(dt),
          alpha_table_(apt),
          theta_table_(tht),
          KinematicInterface(at.size()) {}

    std::vector<State> inverseKinematic(const Eigen::Matrix4d end_effector_pose) override;

    std::unordered_map<std::size_t, Eigen::Matrix4d> forwardKinematic(const State state) override;

    Eigen::VectorXd get_joint(
        Eigen::Vector3d pos_diff, Eigen::VectorXd current_joint_status) override;

    Eigen::Matrix4d get_endeffector_pose(const State& state);

private:
    // 求第一个轴的角度
    void getFirstTheta(float A, float B, float C, float theta1[]);

    // 求腕部的解
    void getWristThetas(const Eigen::Matrix4d pose, float* wrist_solution);

    // 求臂部分的解
    bool getArmThetas(
        float total_theta, std::vector<std::array<float, 3>>& arm_solutions, Eigen::Matrix4d& pose);

    // 参数表
    Ur5eParam a_table_;
    Ur5eParam d_table_;
    Ur5eParam alpha_table_;
    Ur5eParam theta_table_;
};
}  // namespace fast_motion_planning
#endif
