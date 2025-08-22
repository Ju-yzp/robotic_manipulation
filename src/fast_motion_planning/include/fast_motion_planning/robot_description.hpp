#ifndef FAST_MOTION_PLANNING_ROBOT_DESCRIPTION_HPP_
#define FAST_MOTION_PLANNING_ROBOT_DESCRIPTION_HPP_

// yaml parser
#include<yaml-cpp/node/node.h>
#include<yaml-cpp/node/parse.h>
#include<yaml-cpp/yaml.h>

// cpp
#include<type_traits>
#include<string>
#include<iostream>
#include<unordered_map>
#include<vector>
#include<filesystem>
#include<cassert>
#include<cstddef>
#include<memory>

// eigen
#include<Eigen/Eigen>

namespace fast_motion_planning {

template <typename Scalar,
          typename  = std::enable_if_t<std::is_same_v<Scalar, double>|| std::is_same_v<Scalar, float>>>
class RobotDescription
{
public:

using SharedPtr = std::shared_ptr<RobotDescription<Scalar>>;
using UniquePtr = std::unique_ptr<RobotDescription<Scalar>>;

struct JointLimit
{
Scalar joint_position_upper;
Scalar joint_position_lower;
Scalar joint_velocity_upper;
Scalar joint_velocity_lower;
};

struct JointLimitGroup
{
std::unordered_map<std::string,JointLimit> jointlimit_map;
};

struct Envelope
{
Eigen::Vector<Scalar,4> last_position;
Eigen::Vector<Scalar,4> translation;
double radius;
};

struct EnvelopeGroup
{
std::unordered_map<std::string,std::vector<Envelope>> envelope_map_;
};

RobotDescription(const std::string configuration_file)
{
parse_configuration_file(configuration_file);
}

RobotDescription()
{}

void parse_configuration_file(const std::string configuration_file)
{
if(is_initialized_)
   return;

namespace fs = std::filesystem;
if(!fs::exists(configuration_file))
    std::runtime_error("This file that user passed isn't exists ");

YAML::Node params = YAML::LoadFile(configuration_file);
if(!params["robot_params"].IsSequence()){
   std::cerr<<"This joint limits must be a list "<<std::endl;
   return;
}

// 遍历所有关节
for(const auto& joint_msg: params["robot_params"])
{
std::string joint_name = joint_msg["joint"].as<std::string>();
std::cout<<"Joint's name is "<<joint_name<<std::endl;

// 关节线速度和位置限制
JointLimit joint_limit;
joint_limit.joint_position_upper = joint_msg["position_upper"].as<Scalar>();
joint_limit.joint_position_lower = joint_msg["position_lower"].as<Scalar>();
joint_limit.joint_velocity_upper = joint_msg["velocity_upper"].as<Scalar>();
joint_limit.joint_velocity_lower = joint_msg["velocity_lower"].as<Scalar>();

jointlimit_group_.jointlimit_map[joint_name] = joint_limit;

// 包络体
const auto& position_set = joint_msg["envelopes_position"];
const auto& radius_set = joint_msg["envelopes_radius"];

assert(position_set.size() == radius_set.size());

std::vector<Envelope> envelopes;
for(size_t index{0}; index < position_set.size(); ++index)
{
auto const& position = position_set[index].as<std::vector<Scalar>>();
assert(position.size() == 3);
auto radius = radius_set[index].as<Scalar>();
Envelope envelope;
envelope.translation <<position[0],position[1],position[2],1.0;
envelope.radius = radius;
envelopes.emplace_back(envelope);
}
envelope_group_.envelope_map_[joint_name] = envelopes;
}
is_initialized_ = true;
}

JointLimit get_jointlimt(const std::string joint_name)
{
const auto& jointlimits = jointlimit_group_.jointlimit_map;
if(jointlimits.find(joint_name) == jointlimits.end())
  std::cerr<<"No found specfitied jointlimit that have the same joint name"<<std::endl; 

return jointlimit_group_.jointlimit_map[joint_name];
}

std::vector<Envelope>& get_envelopes(const std::string joint_name)
{
const auto& envelopes = envelope_group_.envelope_map_;
if(envelopes.find(joint_name) == envelopes.end())
  std::cerr<<"No found specfitied envelope that have the same joint name"<<std::endl; 

return envelope_group_.envelope_map_[joint_name];
}

bool is_over_joint_velocity_limit(const Scalar value,const std::string joint_name)
{
auto joint_limit = get_jointlimt(joint_name);
return value > joint_limit.joint_veclocity_upper || value < joint_limit.joint_velocity_lower;
}

bool is_over_joint_position_limit(const Scalar value,const std::string joint_name)
{
Scalar new_value = std::fmod(value,2.0f*M_PIf);
new_value = new_value < -M_PIf ? new_value + 2.0f * M_PIf : 
            ((new_value > M_PIf) ? M_PIf - new_value : new_value);

auto joint_limit = get_jointlimt(joint_name);
return new_value > joint_limit.joint_position_upper || new_value < joint_limit.joint_position_lower;
}

private:

bool is_initialized_{false};

EnvelopeGroup envelope_group_;
JointLimitGroup jointlimit_group_;
};
}
#endif