#ifndef MOTION_PLANNING_RRT_PLANNERE_HPP_
#define MOTION_PLANNING_RRT_PLANNERE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>

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
:max_iter_count_(50000),
step_(12.0f),
stop_threshold_(100.0f)
{}

uint32_t get_max_iter_count(){ return max_iter_count_; }

double get_step(){ return step_; }

double get_stop_threshold(){ return stop_threshold_; }

private:
uint32_t max_iter_count_; // 最大迭代次数
double step_; // 步长
double stop_threshold_; // 停止阈值
};

// RRT路径规划器
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

Trajectory<DOF> plan(const Scence &scence,const Eigen::Matrix4f target_pose,const Eigen::Matrix4f source_pose)
{
Trajectory<DOF> trajectory;

bool maybe_succefull{false};
std::cout<<"----Source Pose"<<std::endl;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << source_pose.format(CleanFmt) << std::endl << std::endl;

std::cout<<"----Target Pose"<<std::endl;
std::cout << target_pose.format(CleanFmt) << std::endl << std::endl;

// 先看看能不能到达目标位姿
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(target_pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
maybe_succefull = true;
std::cout<<"Exist solution that make arm reach specficied pose and not collidate with obstacle "<<std::endl;
break;
}
}

// 初步验证可以到达目标位姿，但是具体实现需要
if(maybe_succefull)
{
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(source_pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
root_node = new RRTNode();
root_node->pose = source_pose;
root_node->correct_solution = solution;
tree_.push_back(root_node);
}
}
generate_tree(scence,target_pose,source_pose);
if(successful)
{
std::cout<<"Succefully found a safety reach target pose"<<std::endl;
auto path = get_path();
for(auto iter = path.end() - 1; iter >= path.begin(); iter--)
{
trajectory.plan_tarjectory.push_back((*iter)->correct_solution);
}
}
}

std::cout<<(int)trajectory.plan_tarjectory.size()<<std::endl;
// 释放内存
releaseResource();

return trajectory;
}

private:

struct RRTNode
{
Eigen::Matrix4f pose; // 姿态

std::vector<RRTNode *> children_node_; // 孩子子节点
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
bool successful;  // 是否成功

// 规划器配置信息
PlannerOption option_;

// 随机产生一个位姿，然后检测是否与障碍物发生碰撞
bool generate_random_pose(const Scence& scence,const Eigen::Matrix4f target_pose,const Eigen::Matrix4f source_pose,RRTNode *new_node)
{
Eigen::Matrix4f random_pose;
// 启发式引导
std::random_device rd;
std::mt19937 gen(rd());

const float dx = target_pose(0,3) - source_pose(0,3);
const float dy = target_pose(1,3) - source_pose(1,3);
const float dz = target_pose(2,3) - source_pose(2,3);
const float dist = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));

// 决策分布
std::uniform_real_distribution<float> decision(0.0f,1.0f);
const float choice = decision(gen);
WorkSpace ws = robot_model_->get_workspace();

if( choice < 0.2f)// 目标导向采样
{
random_pose(0,3) = target_pose(0,3);
random_pose(1,3) = target_pose(1,3);
random_pose(2,3) = target_pose(2,3);
}
else if( choice < 0.35f && choice >= 0.2f )// 起点附近采样
{
// 动态调整采样范围（基于迭代次数）
static int iteration = 0;
iteration++;

// 计算最大允许半径（工作空间对角线的10%）
const float max_radius = 0.1f * std::sqrt(
   (ws.bound_max_x - ws.bound_min_x)*(ws.bound_max_x - ws.bound_min_x) +
     (ws.bound_max_y - ws.bound_min_y)*(ws.bound_max_y - ws.bound_min_y) +
     (ws.bound_max_z - ws.bound_min_z)*(ws.bound_max_z - ws.bound_min_z));

// 基础半径随迭代增加
float radius = std::min(0.5f + 0.02f * iteration, max_radius);

std::uniform_real_distribution<float> offset(-radius, radius);
random_pose(0,3) = source_pose(0,3) + offset(gen);
random_pose(1,3) = source_pose(1,3) + offset(gen);
random_pose(2,3) = source_pose(2,3) + offset(gen);
}
else if( choice < 0.6f && choice >= 0.35f)// 路径导向采样
{

// 在起点到目标的连线上采样
std::uniform_real_distribution<float> t_dist(0.0f, 1.0f);
const float t = t_dist(gen);

// 基础点位置
const float base_x = source_pose(0,3) + t * dx;
const float base_y = source_pose(1,3) + t * dy;
const float base_z = source_pose(2,3) + t * dz;

// 随机偏移量（距离的10-30%）
const float offset = (0.1f + 0.2f * decision(gen)) * dist;

// 随机方向（球坐标）
const float theta = 2.0f * M_PI * decision(gen);
const float phi = M_PI * decision(gen);

// 计算偏移向量
const float offset_x = offset * std::sin(phi) * std::cos(theta);
const float offset_y = offset * std::sin(phi) * std::sin(theta);
const float offset_z = offset * std::cos(phi);

// 确保采样点在工作空间内
const float x = std::clamp(base_x + offset_x, ws.bound_min_x, ws.bound_max_x);
const float y = std::clamp(base_y + offset_y, ws.bound_min_y, ws.bound_max_y);
const float z = std::clamp(base_z + offset_z, ws.bound_min_z, ws.bound_max_z);
random_pose(0,3) = x;
random_pose(1,3) = y;
random_pose(2,3) = z;
}
else if( choice < 0.75f && choice >= 0.65f )// 目标附近采样
{
// 动态调整采样范围
static int iteration = 0;
iteration++;

// 计算最大允许半径
const float max_radius = 0.1f * std::sqrt(
   (ws.bound_max_x - ws.bound_min_x)*(ws.bound_max_x - ws.bound_min_x) +
    (ws.bound_max_y - ws.bound_min_y)*(ws.bound_max_y - ws.bound_min_y) +
      (ws.bound_max_z - ws.bound_min_z)*(ws.bound_max_z - ws.bound_min_z));

float radius = std::min(0.5f + 0.02f * iteration, max_radius);

std::uniform_real_distribution<float> offset(-radius, radius);

random_pose(0,3) = target_pose(0,3) + offset(gen);
random_pose(1,3) = target_pose(1,3) + offset(gen);
random_pose(2,3) = target_pose(2,3) + offset(gen);
}
// 工作空间内完全随机采样
else {
std::uniform_real_distribution<float> dist_x(ws.bound_min_x,ws.bound_max_x);
std::uniform_real_distribution<float> dist_y(ws.bound_min_y,ws.bound_max_y);
std::uniform_real_distribution<float> dist_z(ws.bound_min_z,ws.bound_max_z);
random_pose(0,3) = dist_x(gen);
random_pose(1,3) = dist_y(gen);
random_pose(2,3) = dist_z(gen);
}

// 找到最近的节点
RRTNode *nearest_node = nullptr;
// 

double min_distance{INFINITY};
for(auto node:  tree_)
{
double distance = pow(random_pose(0,3)- node->pose(0,3),2)+
                  pow(random_pose(1,3)- node->pose(1,3),2)+
                  pow(random_pose(2,3)- node->pose(2,3),2);

if(min_distance > distance)
{
min_distance = distance;
nearest_node = node;
}
}
// 找到的位姿
Eigen::Matrix4f pose;
// 寻找策略
pose = target_pose;
float artio = sqrt(pow(option_.get_step(),2) / min_distance);
pose(0,3) = nearest_node->pose(0,3) + artio * (random_pose(0,3) - nearest_node->pose(0,3));
pose(1,3) = nearest_node->pose(1,3) + artio * (random_pose(1,3) - nearest_node->pose(1,3));
pose(2,3) = nearest_node->pose(2,3) + artio * (random_pose(2,3) - nearest_node->pose(2,3));


// 逆运动学解算,如果找到有一个对应关节配置下的机械臂与障碍物没有发生碰撞，就返回真
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
nearest_node->children_node_.push_back(new_node);
new_node->parent_node = nearest_node;
new_node->pose = pose;
nearest_node->correct_solution = solution;
// std::cout<<"Found the correct solution"<<std::endl;
return true;
}


return false;
}

return true;
}

void generate_tree(const Scence &scence,const Eigen::Matrix4f target_pose,const Eigen::Matrix4f source_pose)
{
uint32_t iter_count{0};
successful = false;

while (iter_count != option_.get_max_iter_count() ) {

// 挑选距离满足阈值之内的节点
double distance = pow(target_pose(0,3)-tree_[tree_.size()-1]->pose(0,3),2) + 
                  pow(target_pose(1,3)-tree_[tree_.size()-1]->pose(1,3),2) +
                  pow(target_pose(2,3)-tree_[tree_.size()-1]->pose(2,3),2);

if(distance < option_.get_stop_threshold())
{
RRTNode* new_node;
Solutions<DOF> solutions = inverse_kinematic_solver_->inverseKinematic(target_pose);
for(auto& solution:solutions.solutions_)
{
if(!detectCollision(scence,solution))
{
new_node = new RRTNode();
tree_[tree_.size()-1]->children_node_.push_back(new_node);
new_node->parent_node = tree_[tree_.size()-1];
new_node->correct_solution = solution;
new_node->pose = target_pose;
tree_.push_back(new_node);
successful = true;
break;
}
}
}

if(successful)
   break;
// 随机产生位姿
RRTNode *new_node = new RRTNode();
if(generate_random_pose(scence,target_pose,source_pose,new_node))
{
   tree_.push_back(new_node);
}
else 
   delete new_node;

iter_count++;
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
std::cout<<end_node->pose(0,3)<<" "<<end_node->pose(1,3)<<" "<<end_node->pose(2,3)<<std::endl;
path.push_back(end_node);
end_node = end_node->parent_node;
}
path.push_back(root_node);
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