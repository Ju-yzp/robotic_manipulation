// fast motion planning
#include<fast_motion_planning/ur5e_kinematic_solver.hpp>
#include<fast_motion_planning//collision_detector.hpp>

// cpp
#include<iostream>
#include<memory>

int main()
{
namespace fmp = fast_motion_planning;
fmp::RobotDescription<double>::SharedPtr ur5e_description = std::make_shared<fmp::RobotDescription<double>>();
std::string file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/ur5e_description.yaml";
ur5e_description->parse_configuration_file(file);

// TODO：解析解有部分不正确
Ur5eKinematicSolver::Ur5eParam a_table{0.0,0.0,425.0,392.0,0.0,0.0};
Ur5eKinematicSolver::Ur5eParam d_table{163.0,134.0,0.0,0.0,-100.0,100.0};
Ur5eKinematicSolver::Ur5eParam alpha_table{0.0,-M_PI/2.0,0.0,0.0,M_PI/2.0,-M_PI/2.0};
Ur5eKinematicSolver ur5e_kinematic_solver(a_table,d_table,alpha_table,ur5e_description);

ur5e_kinematic_solver.set_joint_name(0, "shoulder_pan");
ur5e_kinematic_solver.set_joint_name(1, "shoulder_lift");
ur5e_kinematic_solver.set_joint_name(2, "elbow");
ur5e_kinematic_solver.set_joint_name(3, "wrist_1_link");
ur5e_kinematic_solver.set_joint_name(4, "wrist_2_link");
ur5e_kinematic_solver.set_joint_name(5, "wrist_3_link");

Ur5eKinematicSolver::Ur5eParam theta{0.2,0.3,0.1,0.2,0.3,1.4};
// ur5e_kinematic_solver.update_envelopes_position(theta);

// fmp::Scene scene;
// scene.obstacles_position.push_back(Eigen::Vector4d{0.0,0.0,0.0,0.0});
// scene.radius.emplace_back(120);

// fmp::CollisionDetector collision_detector(ur5e_description);

// collision_detector.set_name(0, "shoulder_pan");
// collision_detector.set_name(1, "shoulder_lift");
// collision_detector.set_name(2, "elbow");
// collision_detector.set_name(3, "wrist_1_link");
// collision_detector.set_name(4, "wrist_2_link");
// collision_detector.set_name(5, "wrist_3_link");

// if (collision_detector.is_collision_with_obstacles(scene))
//    std::cout<<"Robot arm occur collision with obstacles"<<std::endl;
Eigen::Matrix4d endeffector_pose = ur5e_kinematic_solver.get_endeffector_pose(theta);
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "----Goal Endeffector Pose----" << std::endl;
std::cout << endeffector_pose.format(CleanFmt) << std::endl;

std::cout<<"----Solutions----"<<std::endl;

for(const auto& solution:ur5e_kinematic_solver.solveInverseKinematic(endeffector_pose))
{
std::cout<<"------"<<std::endl;
endeffector_pose = ur5e_kinematic_solver.get_endeffector_pose(solution);
std::cout << solution.format(CleanFmt) << std::endl;
}
return 0;
}