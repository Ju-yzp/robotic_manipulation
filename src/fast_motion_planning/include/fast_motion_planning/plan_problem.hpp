/* PLAN PROBLEM.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月16日 20:07:44
 * Last edited:
 *   YYYY-08-2025年8月17日 08:06:20
 * Auto updated?
 *   Yes
 *
 * Description:
 *   描述运动规划问题
**/

#ifndef FAST_MOTION_PLANNING_PLAN_PROBLEM_HPP_
#define FAST_MOTION_PLANNING_PLAN_PROBLEM_HPP_

#include<Eigen/Eigen>
#include<Eigen/src/Core/Matrix.h>
#include<Eigen/src/Geometry/Quaternion.h>

#include<functional>

namespace fast_motion_planning {
template <typename Scalar,
          typename  = std::enable_if_t<std::is_same_v<Scalar, double>|| std::is_same_v<Scalar, float>>>
class PlanProblem
{
PlanProblem(Eigen::Quaternion<Scalar> start_orient,Eigen::Quaternion<Scalar> goal_orient,
            Eigen::Vector<Scalar,3> start_pos,Eigen::Vector<Scalar,3> goal_pos)
:start_orient_(start_orient),
goal_orient_(goal_orient),
start_pos_(start_pos),
goal_pos_(goal_pos)
{}

PlanProblem(Eigen::Matrix<Scalar,4,4> start,Eigen::Matrix<Scalar,4,4> goal)
{

std::function<Eigen::Quaternion<Scalar>(const Eigen::Matrix<Scalar, 4, 4>)> transfrom = 
[](const Eigen::Matrix<Scalar, 4, 4> pose){
    Eigen::Matrix<Scalar,3,3> rotation_matrix;
    rotation_matrix = pose.block<3,3>(0,0);
    Eigen::Quaternion<Scalar> quaternion(rotation_matrix);
    quaternion.normalize();
    return quaternion;
};

start_orient_ = transfrom(start);
goal_orient_ = transfrom(goal);
start_pos_ << start(0,3),start(1,3),start(2,3);
goal_pos_ << goal(0,3),goal(1,3),goal(2,3);
}

Eigen::Quaternion<Scalar> get_start_oriention(){ return start_orient_; }

Eigen::Quaternion<Scalar> get_goal_oriention(){ return goal_orient_; }

Eigen::Vector<Scalar,3> get_start_position(){ return start_pos_; }

Eigen::Vector<Scalar,3> get_goal_position(){ return goal_pos_; }

private:

// TODO:应该有机械臂这个问题主体才对，但是还没完全理解运动规划问题组成和本质

// 开始位姿和目标位姿
Eigen::Quaternion<Scalar> start_orient_;
Eigen::Vector<Scalar,3> start_pos_;
Eigen::Quaternion<Scalar> goal_orient_;
Eigen::Vector<Scalar,3> goal_pos_;
};
}
#endif
