// cpp
#include<algorithm>
#include<cstddef>
#include<functional>
#include<iostream>
#include<fstream>
// fast motion planning
#include<fast_motion_planning/status_space_manager.hpp>
#include<limits>


namespace fast_motion_planning {
StatusSpaceManager::StatusSpaceManager(double min_x,double max_x,double min_y,double max_y,
                                       double min_z,double max_z)
{
initialize(min_x, max_x, min_y, max_y, min_z, max_z);

space_map_[StatusType::ACTIVE] = get_leaf_nodes();
space_map_[StatusType::INVALID] = std::vector<SpaceNode *>();
space_map_[StatusType::VALID] = std::vector<SpaceNode *>();
}

StatusSpaceManager::~StatusSpaceManager()
{
releaseMemoryResource(root_);
}

void StatusSpaceManager::initialize(double min_x,double max_x,double min_y,double max_y,
                                    double min_z,double max_z)
{
root_ = new SpaceNode();
root_->center(0) = (min_x + max_x) / 2.0;  
root_->center(1) = (min_y + max_y) / 2.0; 
root_->center(2) = (min_z + max_z) / 2.0; 
root_->x_extent =  std::abs(max_x - min_x) / 2.0;
root_->y_extent =  std::abs(max_y - min_y) / 2.0;
root_->z_extent =  std::abs(max_z - min_z) / 2.0;
build(root_);
}

void StatusSpaceManager::build(SpaceNode* const parent)
{
if(parent->x_extent < extent_threshold_ && 
   parent->y_extent < extent_threshold_ &&
   parent->z_extent < extent_threshold_ )
   return ;

for(size_t id{0}; id < 8; ++id)
{
auto& node  = parent->children[id];
const double& x_extent = parent->x_extent / 2.0;
const double& y_extent = parent->y_extent / 2.0;
const double& z_extent = parent->z_extent / 2.0;

node = new SpaceNode();  
node->center(0) = parent->center(0) + ((id & 1) ? x_extent : -x_extent);
node->center(1) = parent->center(1) + ((id >> 1 & 1) ? y_extent : -y_extent);
node->center(2) = parent->center(2) + ((id >> 2 & 1) ? z_extent : -z_extent);
node->x_extent = x_extent ;
node->y_extent = y_extent ;
node->z_extent = z_extent ;
build(node);
}
}

void StatusSpaceManager::releaseMemoryResource(SpaceNode * const parent)
{
if(!parent)
   return ;

for(std::size_t id{0}; id < 8; ++id)
    releaseMemoryResource(parent->children[id]);

delete parent;
}

void StatusSpaceManager::reinitialize(double min_x,double max_x,double min_y,double max_y,
                  double min_z,double max_z)
{
space_map_[StatusType::ACTIVE].clear();
space_map_[StatusType::INVALID].clear();
space_map_[StatusType::VALID].clear();

initialize(min_x, max_x,  min_y,  max_y, min_z,  max_z);
}

// 修正函数名拼写：sreach -> search
SpaceNode* StatusSpaceManager::searchSpaceNode(const Eigen::Vector3d& translation, const SpaceNode* parent) {
    if (!parent) {
        return nullptr;  // 父节点为空，直接返回
    }

    // 检查translation是否在当前节点的空间范围内（包含边界）
    bool in_x = (translation(0) <= parent->center(0) + parent->x_extent) && 
                (translation(0) >= parent->center(0) - parent->x_extent);
    bool in_y = (translation(1) <= parent->center(1) + parent->y_extent) && 
                (translation(1) >= parent->center(1) - parent->y_extent);
    bool in_z = (translation(2) <= parent->center(2) + parent->z_extent) && 
                (translation(2) >= parent->center(2) - parent->z_extent);

    if (!in_x || !in_y || !in_z) {
        // 不在当前节点范围内，返回当前节点（作为边界节点）
        return const_cast<SpaceNode*>(parent);
    }

    // 计算子节点索引（0-7，对应八叉树8个子节点）
    size_t id = (translation(0) > parent->center(0) ? 1 : 0) + 
                (translation(1) > parent->center(1) ? 2 : 0) + 
                (translation(2) > parent->center(2) ? 4 : 0);

    // 若对应子节点存在，递归搜索子节点；否则返回当前节点
    if (parent->children[id]) {
        return searchSpaceNode(translation, parent->children[id]);
    } else {
        return const_cast<SpaceNode*>(parent);
    }
}
std::vector<SpaceNode *> StatusSpaceManager::get_leaf_nodes()
{
std::vector<SpaceNode *> leaf_nodes;

static std::function<void (std::vector<SpaceNode *>& , SpaceNode *)> func = 
[](std::vector<SpaceNode *>& leaf_nodes, SpaceNode *parent)
{
if(!parent->children[0])
{
leaf_nodes.emplace_back(parent);
return ;
}

for(size_t id{0}; id < 8; ++id)
    func(leaf_nodes,parent->children[id]);
};

func(leaf_nodes,root_);

return leaf_nodes;
}

void StatusSpaceManager::update(const Eigen::Vector3d position,bool is_reached)
{
SpaceNode *node = searchSpaceNode(position, root_);
if(!node) return;


// 统计学方差
// static std::function<DistributionType (std::vector<Eigen::Vector3d>&,SpaceNode *)> statistic_func = 
// [](std::vector<Eigen::Vector3d>& points,SpaceNode * node)->decltype(auto)
// {
// // 显示采样点范围
// double min_x = std::numeric_limits<double>::infinity();
// double max_x = std::numeric_limits<double>::lowest();
// double min_y = std::numeric_limits<double>::infinity();
// double max_y = std::numeric_limits<double>::lowest();
// double min_z = std::numeric_limits<double>::infinity();
// double max_z = std::numeric_limits<double>::lowest();

// Eigen::Vector3d average = Eigen::Vector3d::Zero();

// std::for_each(points.begin(),points.end(),[&](const auto& point)
// {
// average += point;
// min_x = point(0) < min_x ? point(0) : min_x;
// max_x = point(0) > max_x ? point(0) : max_x;
// min_y = point(1) < min_y ? point(1) : min_y;
// max_y = point(1) > max_y ? point(1) : max_y;
// min_z = point(2) < min_z ? point(2) : min_z;
// max_z = point(2) > max_z ? point(2) : max_z;
// });

// average /= (double)(points.size() + 1);

// bool x_1 = min_x > node->center(0);
// bool x_2 = max_x > node->center(0);
// bool y_1 = min_y > node->center(1);
// bool y_2 = max_y > node->center(1);
// bool z_1 = min_z > node->center(2);
// bool z_2 = max_z > node->center(2);

// sort(points.begin(),points.end(),[](auto& point1,auto& point2)
// {return point1.norm() > point2.norm();});

// if((x_1&x_2)&&(y_1&y_2)&&(z_1&z_2))
// {
// std::cout<<"集中分布"<<std::endl;
// }
// return DistributionType::LOCAL_CONCENTRATION;
// };

// 只有为活跃状态的节点需要更新,其他类型的代表这个节点所在空间已经充分探索，再继续做采样工作是没意义的
if(node->type == StatusType::ACTIVE)
{

// 根据类型插入对应的容器中
if(is_reached)
   node->valid_points.emplace_back(position);
else node->invalid_points.emplace_back(position);

auto total_count = node->invalid_points.size() + node->valid_points.size();

// 数量达到某个数量阈值后就得开始统计该体素内的元素方差等信息
// 会出现以下情况：
// 1.采样点局部集中
// 2.某些极值干扰(去除噪声)
// 3.
// 我们甚至可以引入一种机制：
// 如果有效采样点和无效采样点待在该体素空间的不同方向上
// TODO: 等待学习相关的统计学知识后写出更加完善的核心代码
if(total_count >= max_capacity_)
   node->type =(double)( node->invalid_points.size()) /(double) (total_count + 1) > 0.4 ?
               StatusType::INVALID : StatusType::INVALID;
}

auto& active_nodes = space_map_[StatusType::ACTIVE];

// 更新映射表
std::for_each(active_nodes.begin(),active_nodes.end(),
[this](SpaceNode *node)
{
    if(node->type != StatusType::ACTIVE)
      space_map_[node->type].emplace_back(node);
});

// 删除在存储活跃节点中的容器已经不是活跃状态的节点
active_nodes.erase(
std::remove_if(active_nodes.begin(),active_nodes.end(),
[&](auto node)
{ return node->type != StatusType::ACTIVE; }),active_nodes.end());
}


void StatusSpaceManager::spaceDivide(SpaceNode * const parent)
{
// TODO:分裂后，需要把之前的采样点重新划分，对于映射表中的
for(std::size_t id{0}; id < 8; ++id)
{
auto& node  = parent->children[id];
node = new SpaceNode();
const double& x_extent = parent->x_extent / 2.0;
const double& y_extent = parent->y_extent / 2.0;
const double& z_extent = parent->z_extent / 2.0;
 
node->center(0) = parent->center(0) + ((id & 1) ? x_extent : -x_extent);
node->center(1) = parent->center(1) + ((id >> 1 & 1) ? y_extent : -y_extent);
node->center(2) = parent->center(2) + ((id >> 2 & 1) ? z_extent : -z_extent);
node->x_extent = x_extent ;
node->y_extent = y_extent ;
node->z_extent = z_extent ;
}

{
// 在映射表通过对内存地址的对比找到的该节点在容器中的位置
// TODO: 通过vector内置函数erase删除后，迭代器已经失效,所以使用{}从形成scope(栈的巧妙之处)根本上杜绝后面错误使用
auto&  container = space_map_[parent->type];
auto iter = container.begin();
for(; iter != container.end(); ++iter)
   if(*iter == parent) { 
      std::cout<<"The address of pointer is "<<*iter<<std::endl;
      container.erase(iter);
      break;}

}

for(std::size_t id{0}; id < 8; ++id)
    space_map_[parent->children[id]->type].emplace_back(parent->children[id]);
}

bool StatusSpaceManager::is_activate(const Eigen::Vector3d& translation)
{
auto node  = searchSpaceNode(translation, root_);
if(!node)
   return false;
for(auto active_node:space_map_[StatusType::ACTIVE])
{
if(node == active_node)
   return true;
}
return false;
}

Eigen::Vector<double,6> StatusSpaceManager::get_space(const Eigen::Vector3d& translation)
{
auto node =searchSpaceNode(translation,root_);
return Eigen::Vector<double,6>{node->center(0),node->center(1),node->center(2),
                               node->x_extent * 1.5,node->y_extent * 1.5,node->z_extent * 1.5};
}
}