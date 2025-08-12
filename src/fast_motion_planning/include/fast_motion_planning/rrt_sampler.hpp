#ifndef FAST_MOTION_PLANNING_RRT_SAMPLER_HPP_
#define FAST_MOTION_PLANNING_RRT_SAMPLER_HPP_

#include<fast_motion_planning/sampler_base_interface.hpp>

#include<cstdint>
#include<mutex>
#include<atomic>
#include<vector>

namespace fast_motion_planning {


class RRTSampler:public SamplerBaseInterface
{
public:

struct RRTNode
{
std::vector<double> solution;
std::vector<RRTNode *> children;
Pose state;
RRTNode* parent;
};

RRTSampler(SamplerOption &option)
:SamplerBaseInterface(option){}

RRTSampler(){
releaseMemoryResource();
}

void plan(const Pose inital_pose,const Pose target_pose);

void sample()override{};

std::atomic<bool>& get_stop_sample_flag(){ return is_stop_sample_;}

std::string name_;

private:

void releaseMemoryResource();

// 开启同步检查的标志
std::atomic<bool> is_stop_sample_{false};

// 锁资源,用于同步检查两个rrt树的节点状态
std::mutex sync_lock_;

// 存储随机生长树的节点和根节点
std::vector<RRTNode *> tree_;
std::vector<RRTNode *> new_nodes_;
RRTNode* root_;

// 迭代次数
uint16_t iter_{0};
};
}
#endif
