#ifndef FAST_MOTION_PALNNING_JOINT_LIMIT_GROUP_HPP_
#define FAST_MOTION_PALNNING_JOINT_LIMIT_GROUP_HPP_

// yaml parser
#include<yaml-cpp/node/node.h>
#include<yaml-cpp/node/parse.h>
#include<yaml-cpp/yaml.h>

// cpp
#include<cstddef>
#include<cstdint>
#include<cmath>
#include<string>
#include<unordered_map>
#include<filesystem>
#include<iostream>
#include<stdexcept>

namespace fast_motion_planning {

enum class JointType:uint8_t
{
UNKNOWN = 0,
CONTINUOUS = 1,
REVOLUTE = 2
};

template <typename Scalar>
class JointLimitGroup
{
public:

struct JointLimit
{
// 关节类型以及关节转动量上下限
JointType joint_type;
Scalar upper_limit;
Scalar lower_limit;
};

JointLimitGroup(std::string config_file)
{
parse_configuration_file(config_file);
}

JointLimitGroup() {}

~JointLimitGroup() {}

bool isOverJointLimit(Scalar value,std::size_t index)
{
if(joint_limit_map_.find(index) == joint_limit_map_.end())
   throw std::runtime_error("This index maybe over range of map");

value = normalize(value);
std::cout<<"new value "<<value<<std::endl;
if(joint_limit_map_[index].joint_type == JointType::CONTINUOUS)
  return false;
if(joint_limit_map_[index].joint_type == JointType::REVOLUTE)
   return (value > joint_limit_map_[index].lower_limit &&
           value < joint_limit_map_[index].upper_limit) ?
           false : true;

if(joint_limit_map_[index].joint_type == JointType::UNKNOWN)
{
std::cout<<"Because type is unknown, we  right by default"<<std::endl;
return false;
}
}

bool isOverJointLimit(Scalar value,std::string joint_name)
{
if(id_map_.find(joint_name) == id_map_.end())
  throw std::runtime_error("This index maybe over range of map");

const size_t id = id_map_[joint_name];
return isOverJointLimit(value,id);

}

void parse_configuration_file(std::string config_file)
{
if(is_initialized_){
   std::cout<<"we had parse configuration file"<<std::endl;
   return;
}

size_t index{0};
namespace fs = std::filesystem;
if(!fs::exists(config_file))
    std::runtime_error("This file that user passed isn't exists ");

YAML::Node config = YAML::LoadFile(config_file);
if(!config["joint_limits"].IsSequence()){
   std::cerr<<"This joint limits must be a list "<<std::endl;
   return;
}

// 遍历所有关节限位
for(const auto& joint_limit:config["joint_limits"])
{
std::string joint_name = joint_limit["joint"].as<std::string>();
id_map_[joint_name] = index;

JointLimit jointLimit;
std::string type = joint_limit["type"].as<std::string>();
if(type == "continuous")
   jointLimit.joint_type = JointType::CONTINUOUS;
else if(type == "revolute")
   jointLimit.joint_type = JointType::REVOLUTE;
else 
   jointLimit.joint_type = JointType::UNKNOWN;

jointLimit.upper_limit =  joint_limit["upper_limit"].as<Scalar>();
jointLimit.lower_limit =  joint_limit["lower_limit"].as<Scalar>();
joint_limit_map_[index] = jointLimit;

index++;
}

is_initialized_ = true;
}

const Scalar normalize(Scalar value)
{
Scalar new_value = std::fmod(value,2.0f*M_PIf);
return new_value = new_value < -M_PIf ? new_value + 2.0f * M_PIf : 
            ((new_value > M_PIf) ? M_PIf - new_value : new_value);
}

private:

// 是否被初始化的标志
bool is_initialized_{false};

// 关节限制映射表
std::unordered_map<std::size_t,JointLimit> joint_limit_map_;

// 索引映射表
std::unordered_map<std::string,std::size_t> id_map_;

};
}

#endif