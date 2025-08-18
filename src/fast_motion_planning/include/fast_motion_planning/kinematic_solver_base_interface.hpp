/* KINEMATIC SOLVER BASE INTERFACE.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月16日 19:17:36
 * Last edited:
 *   YYYY-08-2025年8月16日 22:50:58
 * Auto updated?
 *   Yes
 *
 * Description:
 *   是运动学求解器的基类，提供接口给用户实现具体逆运动学算法
**/

#ifndef FAST_MOTION_PLANNING_KINEMATIC_SOLVER_BASE_INTERFACE_HPP_
#define FAST_MOTION_PLANNING_KINEMATIC_SOLVER_BASE_INTERFACE_HPP_

#include<Eigen/Eigen>
#include<Eigen/src/Core/Matrix.h>
#include<Eigen/src/Geometry/Quaternion.h>

#include<vector>

namespace fast_motion_planning {

class KinematicSolverBaseInterface
{
public:
KinematicSolverBaseInterface(){}

virtual ~KinematicSolverBaseInterface(){};

virtual std::vector<Eigen::VectorXd> solveInverseKinematic(const Eigen::Matrix4d goal_end_effector_pose)  =  0;

virtual std::vector<Eigen::VectorXd> solveInverseKinematic(const Eigen::Quaterniond quaternion,const Eigen::Vector3d translation)
{
Eigen::Matrix4d goal_end_effector_pose = Eigen::Matrix4d::Identity();
goal_end_effector_pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
goal_end_effector_pose.block<3, 1>(0, 3) = translation;
return solveInverseKinematic(goal_end_effector_pose);
}
};
}

#endif

