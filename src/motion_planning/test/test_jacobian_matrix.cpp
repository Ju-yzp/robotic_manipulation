#include <memory>
#include <array>

// cpp
#include <iostream>
#include <cmath>

// motion_planning
#include <motion_planning/mujoco_window.hpp>
#include <motion_planning/inverse_kinematic_solver.hpp>
#include <motion_planning/robotModel.hpp>

int main()
{
// 减少命名空间的部分
using namespace motion_planning;

// ur5e的DH参数，采取改进法进行DH建模
typedef std::array<float,UR5E_DOF> Ur5Param;

// 以弧度制和mm作为基本单位
Ur5Param a{0.0f,0.0f,425.0f,392.0f,0.0f,0.0f};
Ur5Param d{163.0f,134.0f,0.0f,0.0f,-100.0f,100.0f};
Ur5Param alpha{0.0f,-M_PIf/2.0f,0.0f,0.0f,M_PIf/2.0f,-M_PIf/2.0f};
Ur5Param theta{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

// ur5e的关节限位
Ur5Param joint_limit_upper{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
Ur5Param joint_limit_lower{0.0f,0.0f,0.0f,0.0f,0.6f,0.8f};
JointLimit<UR5E_DOF> joint_limit{joint_limit_upper,joint_limit_upper};

// ur5e的机器臂模型
std::shared_ptr<RobotModel<UR5E_DOF>> robot_model = std::make_shared<RobotModel<UR5E_DOF>>(a,d,alpha,theta,joint_limit);
Eigen::Matrix4f origin_pose = robot_model->get_endeffector_status();

// 将机械臂模型与逆运动学求解器关联
std::unique_ptr<InverseKinematicSolver> inverse_kinematic_solver = std::make_unique<InverseKinematicSolver>(robot_model);

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
Eigen::Matrix4f p = robot_model->get_endeffector_status();
std::cout<<"-----Initialize Status-----"<<std::endl;
std::cout << p.format(CleanFmt) << std::endl << std::endl;
Eigen::Vector3f r1 = robot_model->get(1);

std::cout << "----Reducition By Changed Joint1----" << std::endl << std::endl;
r1(0) = r1(0) * 0.08f;
r1(1) = r1(1) * 0.08f;
r1(2) = r1(2) * 0.08f;
std::cout << r1.format(CleanFmt) << std::endl << std::endl;

std::cout << "----Only Change Joint1----" << std::endl << std::endl;
robot_model->set_theta(0.08, 0);
Eigen::Matrix4f pose1 = robot_model->get_endeffector_status();
std::cout << pose1.format(CleanFmt) << std::endl << std::endl;

robot_model->reset_status();
Eigen::Vector3f r2 = robot_model->get(2);
std::cout << "----Reducition By Changed Joint2----" << std::endl << std::endl;
r2(0) = r2(0) * 0.08f;
r2(1) = r2(1) * 0.08f;
r2(2) = r2(2) * 0.08f;
std::cout << r2.format(CleanFmt) << std::endl << std::endl;

std::cout << "----Only Change Joint2----" << std::endl << std::endl;
robot_model->set_theta(0.08f, 1);
Eigen::Matrix4f pose2 = robot_model->get_endeffector_status();
std::cout << pose2.format(CleanFmt) << std::endl << std::endl;

robot_model->reset_status();
Eigen::Vector3f r3 = robot_model->get(3);
std::cout << "----Reducition By Changed Joint3----" << std::endl << std::endl;
r3(0) = r3(0) * 0.08f;
r3(1) = r3(1) * 0.08f;
r3(2) = r3(2) * 0.08f;
std::cout << r3.format(CleanFmt) << std::endl << std::endl;

std::cout << "----Only Change Joint3----" << std::endl << std::endl;
robot_model->set_theta(0.08f, 2);
Eigen::Matrix4f pose3 = robot_model->get_endeffector_status();
std::cout << pose3.format(CleanFmt) << std::endl << std::endl;

robot_model->reset_status();
Eigen::Vector3f r6 = robot_model->get(6);
std::cout << "----Reducition By Changed Joint6----" << std::endl << std::endl;
r6(0) = r6(0) * 0.08f;
r6(1) = r6(1) * 0.08f;
r6(2) = r6(2) * 0.08f;
std::cout << r6.format(CleanFmt) << std::endl << std::endl;

std::cout << "----Only Change Joint6----" << std::endl << std::endl;
robot_model->set_theta(0.08f, 5);
Eigen::Matrix4f pose6 = robot_model->get_endeffector_status();
std::cout << pose6.format(CleanFmt) << std::endl << std::endl;
return 0;
}