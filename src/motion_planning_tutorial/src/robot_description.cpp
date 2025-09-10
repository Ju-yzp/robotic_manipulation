// motion_planning_tutorial
#include <motion_planning_tutorial/robot_description.hpp>

// cpp
#include <cassert>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

// yaml parser
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

namespace motion_planning_tutorial {
RobotDescription::RobotDescription(
    const std::string& configuration_file, KinematicBaseInterface::SharedPtr kinematic_interface)
    : kinematic_interface_(kinematic_interface) {
    parse_configuration_file(configuration_file);
}

void RobotDescription::parse_configuration_file(const std::string& configuration_file) {
    if (is_initialized_) return;

    // 检查配置文件是否存在
    if (!std::filesystem::exists(configuration_file)) {
        std::cerr << "Configuration file does not exist: " << configuration_file << std::endl;
        return;
    }

    try {
        YAML::Node params = YAML::LoadFile(configuration_file);
        if (!params["robot_params"].IsSequence()) {
            std::cerr << "robot_description should be a sequence in configuration file: "
                      << configuration_file << std::endl;
            return;
        }

        // 遍历每一个关节，获取包络体以及约束信息
        for (const auto& joint_information : params["robot_params"]) {
            const std::string joint_name = joint_information["joint"].as<std::string>();
            std::cout << joint_name << std::endl;

            JointLimit joint_limit;
            joint_limit.joint_position_upper = joint_information["position_upper"].as<double>();
            joint_limit.joint_position_lower = joint_information["position_lower"].as<double>();
            joint_limit.joint_velocity = joint_information["velocity"].as<double>();
            joint_limit.joint_angular_acceleration =
                joint_information["angular_acceleration"].as<double>();

            jointlimit_group_.jointlimit_map[joint_name] =
                joint_limit;  // 将关节约束信息存储到关节约束组中

            // 包络体位置集
            const auto& eps = joint_information["envelope_positions"];
            const auto& rs = joint_information["envelope_radius"];

            // 断言判断包络体位置和半径数目是否一致
            assert(
                eps.size() == rs.size() &&
                "envelope_positions and envelope_radius should have the same size");

            std::vector<Envelope> envelopes;  // 包络体向量
            for (size_t id{0}; id < eps.size(); ++id) {
                Envelope envelope;
                auto position = eps[id].as<std::vector<double>>();
                envelope.translation = Eigen::Vector4d(position[0], position[1], position[2], 1.0);
                envelope.radius = rs[id].as<double>();
                envelopes.push_back(envelope);
            }
            envelope_group_.envelope_map_[joint_name] = envelopes;  // 将包络体信息存储到包络体组中
        }

    } catch (const YAML::BadFile& exception) {  // 捕获文件无法打开异常
        std::cerr << "Open configuration file failed: " << exception.what() << std::endl;
        return;
    } catch (const YAML::ParserException& exception) {  // 捕获格式错误
        std::cerr << "Format error in configuration file: " << exception.what() << std::endl;
        return;
    } catch (const YAML::RepresentationException& exception) {  // 捕获节点访问失败
        std::cerr << "Access node failed in configuration file: " << exception.what() << std::endl;
    }
    is_initialized_ = true;
}

void RobotDescription::update_envelopes_position(const State& state) {
    auto transforms = kinematic_interface_->forwardKinematic(state);
    auto& envelope_map = envelope_group_.envelope_map_;
    for (const auto& transform : transforms) {
        const std::string& joint_name = transform.first;
        if (envelope_map.find(joint_name) == envelope_map.end()) {
            std::cerr << "Joint name not found in envelope map: " << joint_name << std::endl;
            continue;
        }  // 更新包络体在机械臂基座坐标系下的位置
        auto& envelopes = envelope_map.at(joint_name);
        for (auto& envelope : envelopes)
            envelope.global_translation = transform.second * envelope.translation;
    }
}

const std::vector<RobotDescription::Envelope> RobotDescription::get_envelope(
    const std::string name) {
    const auto& envelopes = envelope_group_.envelope_map_;
    if (envelopes.find(name) == envelopes.end())
        return std::vector<Envelope>();
    else
        return envelopes.at(name);
}

bool RobotDescription::isOverJointVelocityLimit(
    double joint_velocity, const std::string& joint_name) {
    if (jointlimit_group_.jointlimit_map.find(joint_name) ==
        jointlimit_group_.jointlimit_map.end()) {
        std::cerr << "Joint name not found: " << joint_name << std::endl;
        return false;
    }
    const auto& joint_limit = jointlimit_group_.jointlimit_map.at(joint_name);

    return fabs(joint_velocity) > joint_limit.joint_velocity;
}

bool RobotDescription::isOverJointPositionLimit(
    double joint_position, const std::string& joint_name) {
    if (jointlimit_group_.jointlimit_map.find(joint_name) ==
        jointlimit_group_.jointlimit_map.end()) {
        std::cerr << "Joint name not found: " << joint_name << std::endl;
        return false;
    }
    const auto& joint_limit = jointlimit_group_.jointlimit_map.at(joint_name);

    return (
        joint_position > joint_limit.joint_position_upper ||
        joint_position < joint_limit.joint_position_lower);
}

const RobotDescription::JointLimit RobotDescription::get_jointlimit(const std::string& joint_name) {
    if (jointlimit_group_.jointlimit_map.find(joint_name) ==
        jointlimit_group_.jointlimit_map.end()) {
        std::cerr << "Joint name not found: " << joint_name << std::endl;
        return JointLimit{};  // 返回一个默认的JointLimit对象
    }
    return jointlimit_group_.jointlimit_map.at(joint_name);
}
}  // namespace motion_planning_tutorial
// #include "motion_planning_tutorial/robot_description.hpp"
