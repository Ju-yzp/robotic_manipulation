// fast_motion_planning
#include<fast_motion_planning/rrt_connect_sampler.hpp>

// cpp
#include<thread>
#include<chrono>
#include<atomic>
#include<iostream>

namespace fast_motion_planning {
void RRTConnnectSampler::plan(const Pose inital_pose,const Pose target_pose)
{
RRTSampler lsampler,rsampler;
lsampler.name_ = "fisrt rrtsampler";
rsampler.name_ = "second rrtsampler";
std::thread t1(&RRTSampler::plan,&lsampler,inital_pose,target_pose);
std::thread t2(&RRTSampler::plan,&rsampler,inital_pose,target_pose);
std::thread t3(&RRTConnnectSampler::monitor_rrt_sampler,this,std::ref(lsampler),std::ref(rsampler));

if(t1.joinable())
   t1.join();

if(t2.joinable())
   t2.join();

if(t3.joinable())
   t3.join();
}

void RRTConnnectSampler::monitor_rrt_sampler(RRTSampler& lsampler,RRTSampler& rsampler)
{
while(true)
{
lsampler.get_stop_sample_flag().store(false,std::memory_order_relaxed);
rsampler.get_stop_sample_flag().store(false,std::memory_order_relaxed);
std::cout<<"we prepare to make sampler' to work "<<std::endl;
std::this_thread::sleep_for(std::chrono::milliseconds(20));

lsampler.get_stop_sample_flag().store(true,std::memory_order_relaxed);
rsampler.get_stop_sample_flag().store(true,std::memory_order_relaxed);
std::cout<<"we prepare to stop sampler's sample work RRTConnectSampler managed "<<std::endl;
std::this_thread::sleep_for(std::chrono::milliseconds(20));
}   
}
}
