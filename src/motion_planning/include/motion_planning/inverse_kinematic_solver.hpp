#ifndef MOTION_PLANNING_INVERSE_KINEMATIC_SOLVER_HPP_
#define MOTION_PLANNING_INVERSE_KINEMATIC_SOLVER_HPP_

#include <array>
#include <cstddef>
#include <memory>

#include <motion_planning/inverse_kinematic_base_interface.hpp>
#include <motion_planning/util.hpp>
#include <motion_planning/robotModel.hpp>

namespace motion_planning {

// UR5E是六轴机械臂
const size_t DOF = 6;

class InverseKinematicSolver:public virtual InverseKinematicBaseInterface<DOF>
{
public:

InverseKinematicSolver(std::shared_ptr<RobotModel<DOF>> &robot_model);

~InverseKinematicSolver();

Solutions<DOF> inverseKinematic(Eigen::Matrix4f target_pose) override;

private:

// 求第一个轴的角度
void getFirstTheta(float A,float B,float C,float theta1[]);

// 求腕部的解
void getWristThetas(const Eigen::Matrix4f pose,float *wrist_solution);

// 求臂部分的解
void getArmThetas(float total_theta,std::vector<std::array<float,3>>& arm_solutions,
                  Eigen::Matrix4f &pose);

// 机器人模型
std::shared_ptr<RobotModel<DOF>> robot_model_;

};
}

#endif