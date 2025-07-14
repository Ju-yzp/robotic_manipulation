#ifndef MOTION_PLANNING_ROBOT_MODEL_HPP_
#define MOTION_PLANNING_ROBOT_MODEL_HPP_

#include <array>
#include <cstddef>

#include <iostream>
#include <motion_planning/util.hpp>

#include <Eigen/Eigen>

namespace motion_planning {

template <std::size_t DOF>
class RobotModel
{
public:

RobotModel(std::array<float,DOF> a,std::array<float,DOF> d,
           std::array<float,DOF> alpha,std::array<float,DOF> theta,
           JointLimit<DOF> joint_limit)
:a_(a),d_(d),alpha_(alpha),theta_(theta),joint_limit_(joint_limit)
{}

RobotModel() = delete;

void set_theta(float value,std::size_t index)
{
    theta_[index] = value;
}

float get_a(size_t index){ return a_[index]; }

float get_d(size_t index){ return d_[index]; }

float get_alpha(size_t index){ return alpha_[index]; }

float get_theta(size_t index){ return theta_[index]; }

Eigen::Matrix4f get_endeffector_status()
{
Eigen::Matrix4f endeffector_pose = Eigen::Matrix4f::Identity();
for(size_t index = 0; index < DOF; index++)
{
    endeffector_pose = endeffector_pose *  frameTransform(a_[index],d_[index],alpha_[index], theta_[index]);
}
return endeffector_pose;
}

void selectSolutions(Solutions<DOF> &solutions,const Eigen::Matrix4f target_pose)
{
// int num{0};
// joint_limit_.select_solutions(solutions);
solutions.solutions_.erase(
std::remove_if(solutions.solutions_.begin(),solutions.solutions_.end(),
[&](auto solution)
{ 
for(size_t index = 0; index < DOF; index++)
{
theta_[index] = solution[index];
}
// std::cout<<num++<<std::endl;
Eigen::Matrix4f reach_pose = get_endeffector_status();
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << reach_pose.format(CleanFmt) << std::endl << std::endl;
return !checkEndeffectorPose(target_pose,reach_pose);
}),solutions.solutions_.end());

std::cout<<"Totally had "<<(int)solutions.solutions_.size()<<" vaild solutions"<<std::endl;
}

private:

JointLimit<DOF> joint_limit_;

// dh表，使用改进法
std::array<float,DOF> a_;
std::array<float,DOF> d_;
std::array<float,DOF> alpha_;
std::array<float,DOF> theta_;
};
}
#endif