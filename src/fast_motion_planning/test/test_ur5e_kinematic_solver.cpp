#include<fast_motion_planning/ur5e_kinematic_solver.hpp>

#include<iostream>

int main()
{
// 解析解有部分不正确
Ur5eKinematicSolver::Ur5eParam a_table{0.0,0.0,425.0,392.0,0.0,0.0};
Ur5eKinematicSolver::Ur5eParam d_table{163.0,134.0,0.0,0.0,-100.0,100.0};
Ur5eKinematicSolver::Ur5eParam alpha_table{0.0,-M_PI/2.0,0.0,0.0,M_PI/2.0,-M_PI/2.0};
Ur5eKinematicSolver ur5e_kinematic_solver(a_table,d_table,alpha_table);

Ur5eKinematicSolver::Ur5eParam theta{0.2,0.3,0.5,0.1,0.2,0.7};
Eigen::Matrix4d endeffector_pose = ur5e_kinematic_solver.get_endeffector_pose(theta);
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "----Goal Endeffector Pose----" << std::endl;
std::cout << endeffector_pose.format(CleanFmt) << std::endl;

std::cout<<"----Solutions----"<<std::endl;

for(const auto& solution:ur5e_kinematic_solver.solveInverseKinematic(endeffector_pose))
{
// std::cout<<"------"<<std::endl;
// endeffector_pose = ur5e_kinematic_solver.get_endeffector_pose(solution);
// std::cout << solution.format(CleanFmt) << std::endl;
}
return 0;
}