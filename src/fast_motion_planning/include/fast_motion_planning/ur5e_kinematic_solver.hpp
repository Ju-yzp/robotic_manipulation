/* UR 5E KINEMATIC SOLVER.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月17日 07:57:56
 * Last edited:
 *   YYYY-08-2025年8月17日 07:57:57
 * Auto updated?
 *   Yes
 *
 * Description:
 *   ur5e机械臂运动学解算器，TODO：上臂和前臂求解有些问题
**/

#ifndef FAST_MOTION_PLANNING_UR5E_KINEMATIC_SOLVER_HPP_
#define FAST_MOTION_PLANNING_UR5E_KINEMATIC_SOLVER_HPP_

#include<cstddef>
#include<iostream>

#include<fast_motion_planning/kinematic_solver_base_interface.hpp>

namespace fmp = fast_motion_planning;

const std::size_t UR5E_DOF = 6;

class Ur5eKinematicSolver: public fmp::KinematicSolverBaseInterface
{
public:

using Ur5eParam = Eigen::Vector<double,UR5E_DOF>;

Ur5eKinematicSolver(Ur5eParam a_table,Ur5eParam d_table,Ur5eParam alpha_table)
:a_table_(a_table),
d_table_(d_table),
alpha_table_(alpha_table),
theta_table_(Ur5eParam::Zero())
{
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "----A Table----" << std::endl;
std::cout << a_table.format(CleanFmt) << std::endl;  
std::cout << "----D Table----" << std::endl;
std::cout << d_table.format(CleanFmt) << std::endl;
std::cout << "----Alpha Table----" << std::endl;
std::cout << alpha_table.format(CleanFmt) << std::endl;
}

std::vector<Eigen::VectorXd> solveInverseKinematic(const Eigen::Matrix4d goal_end_effector_pose)override;

Eigen::Matrix4d get_endeffector_pose(Ur5eParam theta_table);

private:

// 求第一个轴的角度
void getFirstTheta(float A,float B,float C,float theta1[]);

// 求腕部的解
void getWristThetas(const Eigen::Matrix4d pose,float *wrist_solution);

// 求臂部分的解
void getArmThetas(float total_theta,std::vector<std::array<float,3>>& arm_solutions,
                  Eigen::Matrix4d &pose);

// Ur5e参数表,DH改进法建模
Ur5eParam a_table_;
Ur5eParam d_table_;
Ur5eParam alpha_table_;
Ur5eParam theta_table_;
};
#endif