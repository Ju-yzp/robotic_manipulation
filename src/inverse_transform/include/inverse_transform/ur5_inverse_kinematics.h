#ifndef ROBOTIC_CONTROL_UR5_INVERSE_KINEMATICS_H__
#define ROBOTIC_CONTROL_UR5_INVERSE_KINEMATICS_H__

#include<inverse_transform/inverse_kinematics_interfaces.h>

#include<array>

using namespace robot_interfaces;

class UR5RobotArm: public robot_interfaces::InverseKinematicsInterface<6>
{
public:
UR5RobotArm();

~UR5RobotArm() = default;

std::vector<Solutions<6>> inverseKinematics(const Pose end_pose)override;

bool isRightSolution(const Solutions<6> solution,const Pose end_pose);


void mutipleSolution(float theta234,float x,float z,float theta5);

std::vector<std::array<float,4>> dh_table_;
std::vector<std::array<float,2>> angle_limit_;
};

#endif