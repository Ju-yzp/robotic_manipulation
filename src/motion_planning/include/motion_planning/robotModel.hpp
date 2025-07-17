#ifndef MOTION_PLANNING_ROBOT_MODEL_HPP_
#define MOTION_PLANNING_ROBOT_MODEL_HPP_

#include <array>
#include <cstddef>

#include <iostream>

#include <memory>
#include <motion_planning/util.hpp>
#include <motion_planning/scence.hpp>

#include <Eigen/Eigen>
#include <vector>

namespace motion_planning {

struct SperhreEnvelope // 球形包络体
{
// 在目标坐标系下的位置
double x,y,z;

// 半径
double radius;

// 在基坐标系下的位置
double transform_x;
double transform_y;
double transform_z;
};

template <std::size_t DOF>
class RobotModel
{
public:

RobotModel(std::array<float,DOF> a,std::array<float,DOF> d,
           std::array<float,DOF> alpha,std::array<float,DOF> theta,
           JointLimit<DOF> joint_limit)
:a_(a),d_(d),alpha_(alpha),theta_(theta),joint_limit_(joint_limit)
{
for(size_t index{0};index < DOF;index++)
    envelopes_[index] = std::vector<SperhreEnvelope>();
}

RobotModel(std::array<float,DOF> a,std::array<float,DOF> d,
           std::array<float,DOF> alpha,std::array<float,DOF> theta)
:a_(a),d_(d),alpha_(alpha),theta_(theta)
{
for(size_t index{0};index < DOF;index++)
    envelopes_[index] = std::vector<SperhreEnvelope>();
}

RobotModel() = delete;

void set_theta(float value,std::size_t index)
{
    theta_[index] = value;
}

void add_envelope(SperhreEnvelope& envelope,std::size_t index) 
{
envelopes_[index].push_back(envelope);
}

float get_a(size_t index){ return a_[index]; }

float get_d(size_t index){ return d_[index]; }

float get_alpha(size_t index){ return alpha_[index]; }

float get_theta(size_t index){ return theta_[index]; }

Eigen::Matrix4f get_endeffector_status()
{
Eigen::Matrix4f endeffector_pose = Eigen::Matrix4f::Identity();
for(size_t index = 0; index < DOF; index++)
{
    endeffector_pose = endeffector_pose *  frameTransform(a_[index],d_[index],alpha_[index], theta_[index]);
}
return endeffector_pose;
}

void selectSolutions(Solutions<DOF> &solutions,const Eigen::Matrix4f target_pose)
{
// int num{0};
// joint_limit_.select_solutions(solutions);
solutions.solutions_.erase(
std::remove_if(solutions.solutions_.begin(),solutions.solutions_.end(),
[&](auto solution)
{ 
for(size_t index = 0; index < DOF; index++)
{
theta_[index] = solution[index];
}
// std::cout<<num++<<std::endl;
Eigen::Matrix4f reach_pose = get_endeffector_status();
// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
// std::cout << reach_pose.format(CleanFmt) << std::endl << std::endl;
return !checkEndeffectorPose(target_pose,reach_pose);
}),solutions.solutions_.end());

std::cout<<"Totally had "<<(int)solutions.solutions_.size()<<" vaild solutions"<<std::endl;
}

bool isOccurCollision(const Scence& scence)
{
Eigen::Matrix4f tranform_matrix = Eigen::Matrix4f::Identity();

// 如果没有障碍物，则一定不与外界发生碰撞
if( scence.get_obstacles().size() == 0)
   return false;

bool flag{false};
// 得到每个包络体在机械臂基底坐标系下的坐标
for(size_t index{0}; index < DOF; index++)
{
// 得到每一帧到基底坐标系的转换矩阵
tranform_matrix = tranform_matrix * frameTransform(a_[index],d_[index],alpha_[index], theta_[index]);
Eigen::Vector4f position;

for(auto &envelope:envelopes_[index])
{
position<<envelope.x,envelope.y,envelope.z,1.0f;
Eigen::Vector4f goal_pos = tranform_matrix * position;
envelope.transform_x = goal_pos(0);
envelope.transform_y = goal_pos(1);
envelope.transform_z = goal_pos(2);

std::cout<<goal_pos(0)<<" "<<goal_pos(1)<<" "<<goal_pos(2)<<std::endl;
// 遍历所有障碍物，看看是否发生碰撞
for(const auto& obstacble_ptr:scence.get_obstacles())
{
// TODO：目前是使用球形障碍物进行算法测试，后面将会使用八叉树表达障碍物的存在
std::shared_ptr<SphereObstacle> ptr = std::dynamic_pointer_cast<SphereObstacle>(obstacble_ptr);
double center_dist = pow(ptr->x-envelope.transform_x,2)+
                     pow(ptr->y-envelope.transform_y,2)+
                     pow(ptr->z-envelope.transform_z,2);

if(sqrt(center_dist) < ptr->radius + envelope.radius){
    std::cout<<"Obstacle occur collision with envelope belongede to the "<<index<<" arm"<<std::endl;
    flag = true;
}
}

}
}

return flag;
}

std::vector<SperhreEnvelope> get_envelope(size_t index)
{
    return envelopes_[index];
}

private:

// 关节限制
JointLimit<DOF> joint_limit_;

// dh表，使用改进法
std::array<float,DOF> a_;
std::array<float,DOF> d_;
std::array<float,DOF> alpha_;
std::array<float,DOF> theta_;

// 球形包络体集
std::map<std::size_t,std::vector<SperhreEnvelope>> envelopes_;
};
}
#endif