#include<fast_motion_planning/envelope_group.hpp>

int main()
{
namespace fmp = fast_motion_planning;
fmp::EnvelopeGroupd envelope_group;

// TODO: 一定要使用文件在文件系统中的绝对路径
std::string file_path = "/home/up/robotics-manipulation/src/fast_motion_planning/config/envelopes.yaml";

envelope_group.parse_configuration_file(file_path);

auto envelopes = envelope_group.get_specfity_link_envelope("upperarm_link");

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

for(auto envelope:envelopes)
{
std::cout<<"-----position-----"<<std::endl;
std::cout << envelope.pos.format(CleanFmt) << std::endl << std::endl;
std::cout<<"-----radius-----"<<std::endl;
std::cout << envelope.radius << std::endl << std::endl;
}
return 0;
}