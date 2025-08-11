#include<fast_motion_planning/rrt_sampler.hpp>

#include<thread>
#include<chrono>
#include<iostream>

namespace fast_motion_planning {
void RRTSampler::plan(const Pose inital_pose,const Pose target_pose)
{
root_ = new RRTNode();
root_->state = inital_pose;

while(iter_ < option_.get_max_iter())
{
if(is_stop_sample_.load(std::memory_order_relaxed)){
   std::this_thread::sleep_for(std::chrono::milliseconds(2));
   std::cout<<name_<<" had stop sampling"<<std::endl;
}
else{
std::this_thread::sleep_for(std::chrono::milliseconds(1));
 std::cout<<name_<<" is sampling"<<std::endl;
iter_++;}
}

std::cout<<name_<<"had over sample count threshold"<<std::endl;
}

void RRTSampler::releaseMemoryResource()
{
// 遍历节点，释放内存
for(auto node:tree_)
{
   if(node != nullptr)
      delete node;
}

// 清理容器，并设置随机生长树根节点为空指针
tree_.clear();
new_nodes_.clear();
root_ = nullptr;
}
}