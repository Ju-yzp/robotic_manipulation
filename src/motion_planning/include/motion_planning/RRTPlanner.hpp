#ifndef MOTION_PLANNING_RRT_PLANNERE_HPP_
#define MOTION_PLANNING_RRT_PLANNERE_HPP_

#include <array>
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
std::vector<array<double,DOF>> plan_tarjectory;
};


// RRT规划器
template <size_t DOF>
class RRTPlanner
{
public:

RRTPlanner(shared_ptr<InverseKinematicBaseInterface<DOF>> &inverse_kinematic_solver)
:inverse_kinematic_solver_(inverse_kinematic_solver),
root_node(nullptr)
{}

~RRTPlanner(){}

Trajectory<DOF> plan(const Scence &scence,const Eigen::Matrix4f target_pose)
{
Trajectory<DOF> trajectory;

generate_tree(scence,target_pose);

// auto path = get_path();
// for(const auto &temp_point:path)
// {
// trajectory.plan_tarjectory.push_back(temp_point);
// }

return trajectory;
}

private:

struct RRTNode
{
Eigen::Matrix4f pose; // 姿态

RRTNode *child_node = nullptr; // 孩子子节点
RRTNode *parent_node = nullptr; // 父节点

std::array<double,DOF> correct_solution;
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

// 随机产生一个位姿，然后检测是否与障碍物发生碰撞
Eigen::Matrix4f generate_random_pose(const Eigen::Matrix4f target_pose)
{
Eigen::Matrix4f random_pose;
// 启发式引导
return random_pose;
}

void generate_tree(const Scence &scence,const Eigen::Matrix4f target_pose)
{
uint32_t iter_count{0};
bool successful{false};

while (iter_count != max_iter_count_ && !successful ) {

Eigen::Matrix4f random_pose = generate_random_pose(target_pose);

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

}
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
while(end_node->parent_node)
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

void detectCollision(const Scence& scence,std::array<double,DOF>)
{

}
};
}
#endif