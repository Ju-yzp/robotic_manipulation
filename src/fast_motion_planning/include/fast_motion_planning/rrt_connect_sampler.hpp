#ifndef FAST_MOTION_PLANNING_RRT_CONNECT_SAMPLER_HPP_
#define FAST_MOTION_PLANNING_RRT_CONNECT_SAMPLER_HPP_

#include<fast_motion_planning/sampler_base_interface.hpp>
#include<fast_motion_planning/rrt_sampler.hpp>

namespace fast_motion_planning {

class RRTConnnectSampler:public SamplerBaseInterface
{
public:
RRTConnnectSampler(SamplerOption &option)
:SamplerBaseInterface(option)
{}

RRTConnnectSampler(){}

void plan(const Pose inital_pose,const Pose target_pose);

void sample()override{};

private:

void monitor_rrt_sampler(RRTSampler& lsampler,RRTSampler& rsampler);

void single_direction_sample();

};
}

#endif