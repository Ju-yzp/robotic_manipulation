// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>

// yaml parser
#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/node/node.h>
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

// cpp
#include <cmath>
#include <iostream>
#include <optional>
#include <utility>

namespace fast_motion_planning {
void RobotParams::parseConfiguration(const std::string config_file) {
    if (is_initialized_) return;

    try {
        YAML::Node params = YAML::LoadFile(config_file);
        if (!params["robot_params"].IsSequence()) {
            std::cerr << "robot_description should be a sequence in configuration file: "
                      << config_file << std::endl;
            return;
        }

        const auto& robot_params = params["robot_params"];

        dof_ = robot_params.size();
        old_state_ = Eigen::VectorXd::Zero(dof_);

        for (size_t id{0}; id < robot_params.size(); ++id) {
            JointLimit jointlimit;
            jointlimit.position_upper = robot_params[id]["position_upper"].as<double>();
            jointlimit.position_lower = robot_params[id]["position_lower"].as<double>();
            jointlimit.velocity = robot_params[id]["velocity"].as<double>();
            jointlimit.acceleration = robot_params[id]["acceleration"].as<double>();
            jointlimit.name = robot_params[id]["joint"].as<std::string>();
            jointlimit_group_[id] = jointlimit;

            Envelopes envelopes;
            static const size_t num = 4;
            const auto& evs = robot_params[id]["envelope_position_radius"];
            envelopes.offset.resize(num, evs.size());
            envelopes.offset_to_base.resize(num, evs.size());
            envelopes.radius.resize(evs.size());

            for (size_t i{0}; i < evs.size(); ++i) {
                const auto prv = evs[i].as<std::vector<double>>();
                if (prv.size() != num) {
                    std::cerr << jointlimit.name << " 's envelope element is incorrect."
                              << std::endl;
                    continue;
                }
                envelopes.offset.block(0, i, num, 1) = Eigen::Vector4d{prv[0], prv[1], prv[2], 1.0};
                envelopes.offset_to_base.block(0, i, num, 1) =
                    Eigen::Vector4d{prv[0], prv[1], prv[2], 1.0};
                envelopes.radius(i) = prv[3];
            }
            envelope_group_[id] = envelopes;
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

bool RobotParams::isOverJointPositionLimit(const size_t id, double value) noexcept {
    if (jointlimit_group_.find(id) == jointlimit_group_.end()) return true;
    normalize(value);

    const auto jointlimit = jointlimit_group_.find(id)->second;
    return jointlimit.position_upper < value || jointlimit.position_lower > value;
}

bool RobotParams::isOverJointVelocity(const size_t id, const double value) noexcept {
    if (jointlimit_group_.find(id) == jointlimit_group_.end()) return true;

    const auto jointlimit = jointlimit_group_.find(id)->second;
    return std::fabs(value) > std::fabs(jointlimit.velocity);
}

void RobotParams::normalize(double& angle) noexcept {
    angle = std::fmod(angle, 2.0 * M_PIf);
    angle = angle > M_PI ? angle - 2.0 * M_PI : (angle < M_PI ? angle + 2.0 * M_PI : angle);
}

std::optional<JointLimit> RobotParams::get_jointlimit(const size_t id) noexcept {
    std::optional<JointLimit> joint_limit;
    if (jointlimit_group_.find(id) == jointlimit_group_.end()) {
        // TODO:这里后面会改成多线程日志库打印调试信息
        std::cerr << "Specified joint limit not found, returning empty value." << std::endl;
        return std::nullopt;
    }

    return joint_limit = jointlimit_group_.find(id)->second;
}

void RobotParams::update_envelope_position(const State state, size_t start, bool is_use_old_value) {
    if (kinematic_solver_) {
        State new_state = Eigen::VectorXd::Zero(dof_);

        if (is_use_old_value) {
            new_state = old_state_;
            for (size_t i{start}; i < state.rows() + start; ++i) new_state(i) = state(i - start);
        } else {
            for (size_t i{start}; i < state.rows() + start; ++i) new_state(i) = state(i - start);
        }

        old_state_ = new_state;

        const auto tf_map = kinematic_solver_->forwardKinematic(new_state);
        for (const auto& tf : tf_map) {
            Envelopes& envelopes = envelope_group_[tf.first];
            envelopes.offset_to_base = tf.second * envelopes.offset;
        }
    } else {
        std::cerr << "Invalid kinematic solver" << std::endl;
        return;
    }
}

std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> RobotParams::get_envelope_position(
    const size_t id) const {
    if (envelope_group_.find(id) == envelope_group_.end()) {
        std::cerr << "Error: No envelope data found for joint '"
                  << "' (index: " << id << ")" << std::endl;
        return std::nullopt;
    }

    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> value;

    std::pair<Eigen::MatrixXd, Eigen::VectorXd> pr_matrix;
    const auto& envelopes = envelope_group_.find(id);
    pr_matrix.first = envelopes->second.offset_to_base;
    pr_matrix.second = envelopes->second.radius;

    return value = pr_matrix;
};

std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> RobotParams::get_envelope_position(
    const std::string joint_name) const {
    size_t id = dof_;

    std::pair<Eigen::MatrixXd, Eigen::VectorXd> pr_matrix;
    for (size_t index{0}; index < dof_; ++index) {
        const auto& jointlimit = jointlimit_group_.find(index)->second;
        if (jointlimit.name == joint_name) {
            id = index;
            break;
        }
    }
    if (dof_ == id) {
        std::cerr << "Error: Joint name '" << joint_name << "' not found." << std::endl;
        return std::nullopt;
    };

    const auto& envelopes = envelope_group_.find(id);
    pr_matrix.first = envelopes->second.offset_to_base;
    pr_matrix.second = envelopes->second.radius;

    return pr_matrix;
}
}  // namespace fast_motion_planning
