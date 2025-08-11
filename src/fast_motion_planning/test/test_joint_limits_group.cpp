#include<fast_motion_planning/joint_limit_group.hpp>

int main()
{
namespace fmp = fast_motion_planning;
std::string file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/joint_limits.yaml";
fmp::JointLimitGroup<double> joint_limit_group(file);

if(joint_limit_group.isOverJointLimit(-3.78f,"shoulder_pan"))
{
std::cout<<"Joint angle user inputed is over it's limit"<<std::endl;
}

return 0;
}