#ifndef FAST_MOTION_PLANNING_ENVELOPE_GROUP_HPP_
#define FAST_MOTION_PLANNING_ENVELOPE_GROUP_HPP_

// eigen
#include<Eigen/Eigen>

// yaml parser
#include<yaml-cpp/node/node.h>
#include<yaml-cpp/node/parse.h>
#include<yaml-cpp/yaml.h>

// cpp
#include<cstddef>
#include<stdexcept>
#include<unordered_map>
#include<vector>
#include<filesystem>
#include<iostream>
#include<string>
#include<cassert>

namespace fast_motion_planning
{

template <typename Scalar>
class EnvelopeGroup
{
public:

struct Envelope
{
Eigen::Vector<Scalar,3> pos;
Scalar radius;
};

EnvelopeGroup(const std::string config_file)
{
parse_configuration_file(config_file);
}

EnvelopeGroup() {};
~EnvelopeGroup() = default;

std::vector<Envelope> get_specfity_link_envelope(const std::size_t index)
{
if(envelope_map_.find(index) == envelope_map_.end())
   std::runtime_error("we don't find link of envelopes that user specfitied");

return envelope_map_[index];
}

std::vector<Envelope> get_specfity_link_envelope(const std::string link_name)
{
if(id_map_.find(link_name) == id_map_.end())
   std::runtime_error("we don't find link of envelopes that user specfitied");

return envelope_map_[id_map_[link_name]];
}

void parse_configuration_file(const std::string config_file)
{
if(is_initialized_){
   std::cout<<"we had parse configuration file"<<std::endl;
   return;
}

size_t count{0};
namespace fs = std::filesystem;
if(!fs::exists(config_file))
    std::runtime_error("This file that user passed isn't exists ");

YAML::Node config = YAML::LoadFile(config_file);
if(!config["links"].IsSequence())
   std::runtime_error("This links must be a list ");

// 遍历所有连杆
for(const auto& link:config["links"])
{
// 如果没有定义link_name属性，或者属性名写错都会引发运行期相关错误抛出
// 这里选择跳过错误
if(!link["link_name"].IsDefined())
{
std::cerr<<"Skip auto wrong link"<<std::endl;
continue;
}
std::string link_name = link["link_name"].as<std::string>();
envelope_map_[count] = std::vector<Envelope>();
id_map_[link_name] = count;

//std::cout<<__FILE__<<__LINE__<<std::endl;
assert(link["position"].size() == link["radius"].size());

const auto& position_vector =  link["position"];
const auto& radius_vector =  link["radius"];
for(size_t index{0}; index <position_vector.size(); index++)
{
Envelope envelope;
auto position = position_vector[index].as<std::vector<Scalar>>();
auto radius = radius_vector[index].as<Scalar>();

envelope.pos << position[0],position[1],position[2];
envelope.radius = radius;
envelope_map_[count].push_back(envelope);
}
std::cout<<"Link's name is "<<link_name<<std::endl;

count++;
}

is_initialized_ = true;
}

private:

bool is_initialized_{false}; 

// 包络体集合信息
std::unordered_map<std::size_t,std::vector<Envelope>> envelope_map_;
std::unordered_map<std::string,std::size_t> id_map_;
};

typedef EnvelopeGroup<double> EnvelopeGroupd;
typedef EnvelopeGroup<float> EnvelopeGroupf;
}
#endif