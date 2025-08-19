// cpp
#include<algorithm>
#include<cstddef>
#include<functional>
#include<iostream>

// fast motion planning
#include<fast_motion_planning/status_space_manager.hpp>


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

SpaceNode* StatusSpaceManager::sreachSpaceNode(const Eigen::Vector3d translation,const SpaceNode * const parent)
{
if(!parent)
   return nullptr;

if(translation(0) < parent->center(0) + parent->x_extent && 
   translation(0) > parent->center(0) - parent->x_extent &&
   translation(1) < parent->center(1) + parent->y_extent &&
   translation(1) > parent->center(1) - parent->y_extent &&
   translation(2) < parent->center(2) + parent->z_extent &&
   translation(2) > parent->center(2) - parent->z_extent)
{
size_t id{0};
if(!parent->children[id])
   return const_cast<SpaceNode *>(parent);

id = (translation(0) > parent->center(0) ? 1 : 0) + 
     (translation(1) > parent->center(1) ? 2 : 0) + 
     (translation(2) > parent->center(2) ? 4 : 0) ;
return sreachSpaceNode(translation, parent->children[id]);
}else { return nullptr; }
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
SpaceNode *node = sreachSpaceNode(position, root_);
if(!node) return;

// 统计学方差
static std::function<DistributionType (std::vector<Eigen::Vector3d>&)> statistic_func = 
[](std::vector<Eigen::Vector3d>& points)->decltype(auto)
{
return DistributionType::LOCAL_CONCENTRATION;
};

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
if(total_count >= max_capacity_ * 0.75)
{

}
}

auto& active_nodes = space_map_[StatusType::ACTIVE];

// 更新映射表
std::for_each(active_nodes.begin(),active_nodes.end(),
[this](SpaceNode *node)
{
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

}