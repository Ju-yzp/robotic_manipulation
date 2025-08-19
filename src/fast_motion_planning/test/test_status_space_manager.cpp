#include<fast_motion_planning/status_space_manager.hpp>

#include<iostream>

int main()
{
namespace fmp =fast_motion_planning;
fmp::StatusSpaceManager status_space_manager(-500.0,500.0,-500.0,500.0,50.0,700.0);

// TODO: 测试空间分裂函数，调用时请将类权限改为public

// auto node = status_space_manager.sreachSpaceNode(Eigen::Vector3d{101.6,90.7,200.8}, status_space_manager.root_);

// std::cout<<"The number of active node is "<<(int)(status_space_manager.get_active_space().size())<<" before node devide"<<std::endl;
// if(node)
//   std::cout<<"it's a pointer that has memory resource "<<std::endl;
// status_space_manager.spaceDivide(node);
// std::cout<<"The address of pointer is "<<node<<std::endl;
// std::cout<<"The number of active node is "<<(int)(status_space_manager.get_active_space().size())<<" after node devide"<<std::endl;
return 0;
}