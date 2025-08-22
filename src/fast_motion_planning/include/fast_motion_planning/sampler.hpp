/* SAMPLER.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月19日 09:10:18
 * Last edited:
 *   YYYY-08-2025年8月21日 23:48:38
 * Auto updated?
 *   Yes
 *
 * Description:
 *   采样器，用于测试状态空间的性能，后面会重新设计,其实这里更像是规划器（控制采样器，碰撞检测器，逆运动学解算器）
**/

#ifndef FAST_MOTION_PLANNING_SAPLER_HPP_
#define FAST_MOTION_PLANNING_SAPLER_HPP_

// fast_motion_planning
#include<fast_motion_planning/status_space_manager.hpp>
#include<fast_motion_planning/plan_problem.hpp>
#include<fast_motion_planning/collision_detector.hpp>
#include<fast_motion_planning/kinematic_solver_base_interface.hpp>

// cpp
#include<functional>
#include<memory>
#include<vector>
#include<tuple>

namespace fast_motion_planning {

struct Pose
{
Eigen::Matrix4d toMatrix()
{
Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
transform.block<3,3>(0,0) = orientation.toRotationMatrix();
transform(0,3) = translation(0);
transform(1,3) = translation(1);
transform(2,3) = translation(2);
return transform;
}
Eigen::Vector3d translation;     // 位移
Eigen::Quaterniond  orientation; // 朝向
};


class Sampler
{
public:

struct RRTNode
{
Pose pose;
RRTNode *parent = nullptr;
std::vector<Eigen::VectorXd> valid_solutions;
};

using WeightFunc = std::function<Eigen::VectorXd(std::vector<Eigen::VectorXd>&,const Eigen::VectorXd&)>;

Sampler(StatusSpaceManager::UniquePtr& status_space_manager,const PlanProblem<double>& problem,
        std::unique_ptr<CollisionDetector>& collision_detector,
        std::shared_ptr<KinematicSolverBaseInterface> kinematic_solver,
        double step,size_t max_sample_count);

~Sampler(){ releaseMemorySource(); };

void plan();

// 重新设置路径规划问题，但这个版本是针对静态环境的
void reset_plan_problem(const PlanProblem<double>& problem);

// 如果失败，会返回一个空集合
std::vector<Eigen::VectorXd> get_path();

// 一定进行权重函数的绑定的话，如果没有进行绑定操作，默认使用第一个解
void bindWeightFunction(WeightFunc func){weight_func_ = func;}

private:

void update_nearest_node(RRTNode* node)
{
nearest_node_ = (nearest_node_->pose.translation - problem_.get_goal_position()).norm() > 
                (node->pose.translation - problem_.get_goal_position()).norm() ?
                 node : nearest_node_;
}

Pose sample();

void update(Pose& pose,bool is_reached);

void releaseMemorySource();

RRTNode* searchNearestNode(const Eigen::Vector3d&translation);

std::tuple<bool,RRTNode *> isFreeCollision(Pose& pose);

// 状态空间管理者
StatusSpaceManager::UniquePtr status_space_manager_;

// 规划问题
PlanProblem<double> problem_;

// rrt树和根节点
std::vector<RRTNode *> tree_;
RRTNode* root_ = nullptr;

// 碰撞检测器
std::unique_ptr<CollisionDetector> collision_detector_;

// 逆运动学解算器
std::shared_ptr<KinematicSolverBaseInterface> kinematic_solver_;

// 最大采样次数
size_t max_sample_count_;

// 步长 
double step_;

// 权重函数
WeightFunc weight_func_;

// 距离最近的根节点
RRTNode *nearest_node_ = nullptr;
};

}

#endif