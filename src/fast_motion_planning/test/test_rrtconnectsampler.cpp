#include<fast_motion_planning/rrt_connect_sampler.hpp>

int main()
{
namespace fmp = fast_motion_planning;
fmp::RRTConnnectSampler rrt_connect_sampler;
fmp::Pose inital_pose,target_pose;
rrt_connect_sampler.plan(inital_pose,target_pose);
return  0;
}