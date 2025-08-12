#ifndef FAST_MOTION_PALNNING_KINEMATIC_BASE_INTERFACE_HPP_
#define FAST_MOTION_PALNNING_KINEMATIC_BASE_INTERFACE_HPP_

// eigen
#include<Eigen/Eigen>
#include<Eigen/src/Core/Matrix.h>
#include<Eigen/src/Core/util/Constants.h>

// cpp
#include<array>
#include<cmath>
#include<cstddef>
#include<iostream>
#include<type_traits>

// fast_motion_planning
#include<fast_motion_planning/sampler_base_interface.hpp>

namespace fast_motion_planning {

template <std::size_t DOF,typename Scalar>
struct Thetas
{
std::array<Scalar,DOF> solutions;
};

template <std::size_t DOF,typename Scalar,
          typename  = std::enable_if_t<std::is_same_v<Scalar, double>|| std::is_same_v<Scalar, float>>>
class KinematicBaseInterface
{
public:
using Pose = Eigen::Matrix<Scalar,4,4>;
using TF = Eigen::Matrix<Scalar,4,4>;
using DynamicMatrix = Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic>;

KinematicBaseInterface(){}

virtual ~KinematicBaseInterface(){}

virtual std::array<Scalar,DOF> solveInverseTransform(const Pose target_pose) = 0;

// 使用的是DH改进法表示帧变换
static TF frameTransform(Scalar alpha,Scalar a,Scalar d,Scalar theta)
{
Scalar st = sin(theta);
Scalar ct = cos(theta);
Scalar sa = sin(alpha);
Scalar ca = cos(alpha);

TF transfrom_matrix;
transfrom_matrix<<ct,   -st,  0,  a,
                  st*ca,ct*ca,-sa,-sa*d,
                  st*sa,ct*sa,ca, ca*d,
                  0,    0,    0,  0;

return transfrom_matrix;
}

// 输入表索引，然后输出帧变换
TF frameTransform(const size_t id)
{
if(id < DOF)
    return frameTransform(alpha_table_[id],a_table_[id],d_table_[id],theta_table_[id]);
std::cerr<<"Index user inputed had over range "<<std::endl;
return TF::Identity();
}

// 输出该机械臂模型的雅各比矩阵
DynamicMatrix get_jacobian_matrix()
{
for(size_t id{0}; id < DOF; id++)
{

}
}

protected:

// 上一个关节状态
std::array<Scalar,DOF> last_status_;

// 使用改进DH法表达机械臂模型
std::array<Scalar,DOF> alpha_table_;
std::array<Scalar,DOF> theta_table_;
std::array<Scalar,DOF> a_table_;
std::array<Scalar,DOF> d_table_;
};
}
#endif