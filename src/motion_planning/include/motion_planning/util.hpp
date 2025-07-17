#ifndef MOTION_PLANNING_UTIL_HPP_
#define MOTION_PLANNING_UTIL_HPP_

#include <algorithm>
#include <cstddef>
#include <array>
#include <vector>

#include <Eigen/Eigen>

namespace motion_planning {

template <std::size_t DOF>
struct Solutions
{
std::vector<std::array<float,DOF>> solutions_;
};

template <std::size_t DOF>
struct JointLimit
{
void select_solutions(Solutions<DOF> &solutions)
{
solutions.solutions_.erase(
std::remove_if(solutions.solutions_.begin(),solutions.solutions_.end(),
[&](auto solution)
{ 
for(size_t index = 0; index < DOF; index++)
{
if(solution[index] < lower_[index] || solution[index] > upper_[index])
return true;
}
return false;
}),solutions.solutions_.end());

}

std::array<float,DOF> upper_; // 上限
std::array<float,DOF> lower_; // 下限

};

Eigen::Matrix4f frameTransform(float a,float d,float alpha,float theta);

bool checkEndeffectorPose(const Eigen::Matrix4f target_pose,const Eigen::Matrix4f reach_pose);
}

#endif