/* STATUS SPACE MANAGER.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月18日 23:07:06
 * Last edited:
 *   YYYY-08-2025年8月22日 02:01:48
 * Auto updated?
 *   Yes
 *
 * Description:
 *   状态空间管理器. 推荐在机械臂工作的静态环境中使用，动态环境也可以（可以利用之前的探索记录）
 *   避免对空间进行不必要的探索，同时之前的探索工作作为一种先验存在
**/


#ifndef FAST_MOTION_PLANNING_STATUS_SPACE_MANAGER_HPP_
#define FAST_MOTION_PLANNING_STATUS_SPACE_MANAGER_HPP_

// cpp
#include<cstdint>
#include<unordered_map>
#include<vector>
#include<memory>

// eigen
#include<Eigen/Eigen>


namespace fast_motion_planning {
// 空间的状态类型
enum class StatusType:uint8_t
{
ACTIVE = 0,  // 未对空间进行充分探索，活跃状态
INVALID = 1, // 对空间已经进行充分探索，但是无效采样点占据大部分空间
VALID = 2,   // 对空间已经进行充分探索，而且有效采样点占据大部分空间
};

struct SpaceNode
{
// 因为机械臂与障碍物发生碰撞或者机械臂尺寸限制无法到达而产生的无效点
std::vector<Eigen::Vector3d> invalid_points;
// 机械臂可以到达且与障碍物不发生碰撞的有效点
std::vector<Eigen::Vector3d> valid_points;
// 立方体空间中心
Eigen::Vector3d center;
// 从中心出发，每个轴方向上到达边界的距离
double x_extent;
double y_extent;
double z_extent;
// 子节点
SpaceNode *children[8] = {nullptr};
// 节点状态
StatusType type{StatusType::ACTIVE};
};

class StatusSpaceManager
{
public:

using UniquePtr = std::unique_ptr<StatusSpaceManager>;
using SharedPtr = std::shared_ptr<StatusSpaceManager>;

// 用户必须提供不精确的工作空间数据（呈立方体）
explicit StatusSpaceManager(double min_x,double max_x,double min_y,double max_y,
                            double min_z,double max_z);

// 析构对象时，必须释放申请的内存资源
~StatusSpaceManager();

// 获取活跃空间节点
std::vector<SpaceNode *> get_active_space(){ return space_map_[StatusType::ACTIVE]; };

// 获取无效空间节点
std::vector<SpaceNode *> get_invalid_space(){ return space_map_[StatusType::INVALID]; };

// 获取有效空间节点
std::vector<SpaceNode *> get_valid_space(){ return space_map_[StatusType::VALID]; };

// 重新进行初始化工作
void reinitialize(double min_x,double max_x,double min_y,double max_y,
                  double min_z,double max_z);

// 根据采样器的反馈来更新状态空间
void update(const Eigen::Vector3d position,bool is_reached = true);

void update(const Eigen::Vector3d position,const SpaceNode *node,bool is_reached = true);

std::vector<SpaceNode *> get_leaf_nodes();

bool is_activate(const Eigen::Vector3d& translation);

Eigen::Vector<double,6> get_space(const Eigen::Vector3d& translation);

private:

// 采样点的分布情况
enum class DistributionType:uint8_t
{
LOCAL_CONCENTRATION = 0, // 局部集中
UNIFORM_DISTRIBUTION = 1 // 均匀分布
};

// 初始化，供构造函数以及重新初始化函数使用
void initialize(double min_x,double max_x,double min_y,double max_y,
                double min_z,double max_z);

// 构建八叉树
void build(SpaceNode* const parent);

// 释放申请的内存
void releaseMemoryResource(SpaceNode * const parent);

// 空间节点分裂（被调用的情况，无效采样点和有效采样点都局部集中在该体素空间所属子体素中，且大部分并不混淆)
void spaceDivide(SpaceNode * const parent);

// TODO:一般传入的都是根节点
SpaceNode* searchSpaceNode(const Eigen::Vector3d& translation,const SpaceNode * const parent);

// 根节点
SpaceNode *root_ = nullptr;

// 映射表管理空间
std::unordered_map<StatusType, std::vector<SpaceNode *>> space_map_;

// 空间节点最大容量
std::size_t max_capacity_{500};

// 划分空间时，每个节点空间在每个轴上的长度的一半都得小于这个阈值
double extent_threshold_{100.0};

// 最小长度，重新划分一个节点空间时，每个轴上的长度都得大于它
double min_extent_{30};


};
}
#endif 