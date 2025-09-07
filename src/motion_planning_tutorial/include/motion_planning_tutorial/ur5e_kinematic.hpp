/*
Description: Ur5eKinametic: a class for describing ur5e robot kinematic interface
Author: Jup email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_UR5E_KINEMATIC_HPP_
#define MOTION_PLANNING_TOTURIAL_UR5E_KINEMATIC_HPP_

// cpp
#include <array>
#include <cstddef>
#include <unordered_map>

// motion_planning_tutorial
#include <motion_planning_tutorial/kinematic_base_interface.hpp>

namespace motion_planning_tutorial {

using Ur5eParam = std::array<double, 6>;
class Ur5eKinematic : virtual public KinematicBaseInterface {
public:
    Ur5eKinematic(const Ur5eParam at, const Ur5eParam dt, const Ur5eParam apt, const Ur5eParam tht)
        : a_table_(at), d_table_(dt), alpha_table_(apt), theta_table_(tht) {}

    void set_id(const size_t id, const std::string joint_name) { id_map_[id] = joint_name; }

    std::vector<State> inverseKinematic(const Eigen::Isometry3d& goal_pose) override;

    std::unordered_map<std::string, Eigen::Isometry3d> forwardKinematic(
        const State& state) override;

    Eigen::Isometry3d get_endeffector_pose(const State& state);

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

    // 映射表
    std::unordered_map<size_t, std::string> id_map_;
};
}  // namespace motion_planning_tutorial
#endif
