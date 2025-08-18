#include<fast_motion_planning/robot_description.hpp>

int main()
{
namespace fmp = fast_motion_planning;
fmp::RobotDescription<double> ur5e_description;
std::string file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/ur5e_description.yaml";
ur5e_description.parse_configuration_file(file);
std::string joint_name = "shoulder_pan";
std::cout<<ur5e_description.get_jointlimt(joint_name).joint_position_lower<<std::endl;

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "Printf Envelope" << std::endl << std::endl;
for(const auto& envelope:ur5e_description.get_envelopes(joint_name))
{
std::cout << envelope.format(CleanFmt) << std::endl << std::endl;; 
}

if(ur5e_description.is_over_joint_position_limit(2.0f, joint_name))
{
std::cout<<"This over position limit"<<std::endl;
}
}