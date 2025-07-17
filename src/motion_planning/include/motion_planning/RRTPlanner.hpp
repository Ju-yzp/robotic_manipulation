#ifndef MOTION_PLANNING_RRT_PLANNERE_HPP_
#define MOTION_PLANNING_RRT_PLANNERE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <motion_planning/robotModel.hpp>
#include <motion_planning/inverse_kinematic_base_interface.hpp>
#include <motion_planning/util.hpp>
#include <motion_planning/scence.hpp>

#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/Dense>

namespace motion_planning {

using namespace std;

template <size_t DOF>
struct Trajectory
{
std::vector<array<float,DOF>> plan_tarjectory;
};


class PlannerOption // 规划器配置
{
public:
PlannerOption(uint32_t max_iter_count,double step,double stop_threshold)
:max_iter_count_(max_iter_count),
step_(step),
stop_threshold_(stop_threshold)
{}

PlannerOption()
:max_iter_count_(40000),
step_(7.0f),
stop_threshold_(3.0f)
{}

uint32_t get_max_iter_count(){ return max_iter_count_; }

double get_step(){ return step_; }

double get_stop_threshold(){ return stop_threshold_; }

private:
uint32_t max_iter_count_; // 最大迭代次数
double step_; // 步长
double stop_threshold_; // 停止阈值
};

// RRT规划器
template <size_t DOF>
class RRTPlanner
{
public:

RRTPlanner(shared_ptr<InverseKinematicBaseInterface<DOF>> inverse_kinematic_solver,
           shared_ptr<RobotModel<DOF>>& robot_model,
           PlannerOption &option)
:inverse_kinematic_solver_(inverse_kinematic_solver),
robot_model_(robot_model),
option_(option),
root_node(nullptr)
{}

RRTPlanner(shared_ptr<InverseKinematicBaseInterface<DOF>> inverse_kinematic_solver,
           shared_ptr<RobotModel<DOF>>& robot_model)
:inverse_kinematic_solver_(inverse_kinematic_solver),
robot_model_(robot_model),
option_(PlannerOption()),
root_node(nullptr)
{}

// 不允许没有逆运动学求解器和机器人模型
RRTPlanner() = delete;

~RRTPlanner(){}

Trajectory<DOF> plan(const Scence &scence,const Eigen::Matrix4f target_pose)
{
Trajectory<DOF> trajectory;

bool maybe_succefull{false};

// 先看看能不能到达目标位姿
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(target_pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
maybe_succefull = true;
std::cout<<"Exist solution that make arm reach specficied pose and not collidate with obstacle "<<std::endl;
}
}

std::cout<<"Failedd to find a correct solution make arm avoid the obstacles"<<std::endl;

// 初步验证可以到达目标位姿，但是具体实现需要
if(maybe_succefull)
{
generate_tree(scence,target_pose);
}
 
return trajectory;
}

private:

struct RRTNode
{
Eigen::Matrix4f pose; // 姿态

RRTNode *child_node = nullptr; // 孩子子节点
RRTNode *parent_node = nullptr; // 父节点

std::array<float,DOF> correct_solution;
};

// 逆运动学解算器
shared_ptr<InverseKinematicBaseInterface<DOF>> inverse_kinematic_solver_;

// 机器人模型
shared_ptr<RobotModel<DOF>> robot_model_;

// rrt
vector<RRTNode *> tree_; // 记录树的节点
RRTNode *root_node; // 树的根节点
uint32_t max_iter_count_; // 最大迭代次数
bool successful;  // 是否成功
double step_; // 步长
double stop_threshold_; // 停止阈值

// 规划器配置信息
PlannerOption option_;

// 随机产生一个位姿，然后检测是否与障碍物发生碰撞
bool generate_random_pose(const Scence& scence,const Eigen::Matrix4f target_pose,RRTNode *new_node = nullptr)
{
Eigen::Matrix4f random_pose;
// 启发式引导

// 找到最近的节点
RRTNode *nearest_node = nullptr;

// 
double min_distance;
for(auto node:  tree_)
{
double distance = pow(random_pose(0,3)- node->pose(0,3),2)+
                  pow(random_pose(1,3)- node->pose(1,3),2)+
                  pow(random_pose(2,3)- node->pose(2,3),2);

if(min_distance < distance)
{
min_distance = distance;
nearest_node = node;
}

// 找到的位姿

Eigen::Matrix4f pose;
// 寻找策略

// 逆运动学解算,如果找到有一个对应关节配置下的机械臂与障碍物没有发生碰撞，就返回真
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
new_node = new RRTNode{pose,nearest_node,nullptr,solution};
return true;
break;
}
}

return false;
}

return true;
}

void generate_tree(const Scence &scence,const Eigen::Matrix4f target_pose)
{
uint32_t iter_count{0};
bool successful{false};

while (iter_count != max_iter_count_ && !successful ) {

// 挑选距离满足阈值之内的节点
double distance = pow(target_pose(0,3)-tree_[tree_.size()-1]->pose(0,3),2) + 
                  pow(target_pose(1,3)-tree_[tree_.size()-1]->pose(1,3),2) +
                  pow(target_pose(2,3)-tree_[tree_.size()-1]->pose(2,3),2);

if(distance < stop_threshold_)
{
RRTNode* new_node;
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(target_pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
new_node = new RRTNode{target_pose,tree_[tree_.size()-1],nullptr,solution};
successful = true;
break;
}
}
}

// 随机产生位姿
RRTNode *new_node;
if(generate_random_pose(scence,target_pose))
   tree_.push_back(new_node);
else 
{
   iter_count++;
   continue;
}
}
}

// 返回路径,如果没有元素，则反映没有找到安全避开障碍物的
vector<RRTNode *> get_path()
{
vector<RRTNode *> path;

if(!successful)
   return path;

RRTNode *end_node = tree_[tree_.size()-1];
while(end_node->parent_node && (end_node != root_node ))
{
tree_.push_back(end_node);
end_node = end_node->parent_node;
}
tree_.push_back(root_node);
return path;
}

// 释放资源
void releaseResource()
{
for(auto node:tree_)
{
delete node;
}
root_node = nullptr;
}

bool detectCollision(const Scence& scence,std::array<float,DOF>& solution)
{
for(size_t index{0}; index  < DOF; index++)
{
robot_model_->set_theta(solution[index],index);
}

if(robot_model_->isOccurCollision(scence))
   return true;

return false;
}
};
}
#endif