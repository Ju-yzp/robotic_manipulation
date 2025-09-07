/*
Description: RobotDescription: a class for describing robot consisting of kinematic interface, joint
Author: Jup email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_ROBOT_DESCRIPTION_HPP_
#define MOTION_PLANNING_TOTURIAL_ROBOT_DESCRIPTION_HPP_

// cpp
#include <memory>
#include <string>
#include <unordered_map>

// eigen
#include <eigen3/Eigen/Eigen>

// motion_planning_tutorial
#include <motion_planning_tutorial/kinematic_base_interface.hpp>

namespace motion_planning_tutorial {
class RobotDescription {
public:
    using SharedPtr = std::shared_ptr<RobotDescription>;
    using UniquePtr = std::unique_ptr<RobotDescription>;

    struct JointLimit {               // 关节约束
        double joint_position_upper;  // 关节位置上限
        double joint_position_lower;  // 关节位置下限
        double joint_velocity_upper;  // 关节速度上限
        double joint_velocity_lower;  // 关节速度下限
    };

    struct Envelope {                 // 包络体
        Eigen::Vector4d translation;  // 包络体在关节坐标系下的位置
        Eigen::Vector4d global_translation =
            Eigen::Vector4d{0.0, 0.0, 0.0, 1.0};  // 包络体在基座坐标系下的位移
        double radius;                            // 半径
    };

    RobotDescription(
        const std::string& configuration_file,
        KinematicBaseInterface::SharedPtr kinematic_interface);

    // 解析配置文件
    void parse_configuration_file(const std::string& configuration_file);

    // 更新包络体在机械臂基座坐标系下的位置
    void update_envelopes_position(const State& state);

    const std::vector<Envelope> get_envelope(const std::string name);

    // 判断是否超过关节角速度约束
    bool isOverJointVelocityLimit(double joint_velocity, const std::string& joint_name);

    // 判断是否超过关节位置约束
    bool isOverJointPositionLimit(double joint_position, const std::string& joint_name);

    // 获取关节约束
    const JointLimit get_jointlimit(const std::string& joint_name);

private:
    struct EnvelopeGroup {  // 包络体组，是位于同一个关节下的
        std::unordered_map<std::string, std::vector<Envelope>> envelope_map_;
    };

    struct JointLimitGroup {                                         // 关节约束组
        std::unordered_map<std::string, JointLimit> jointlimit_map;  // 关节名和关节约束的映射
    };

    EnvelopeGroup envelope_group_;
    JointLimitGroup jointlimit_group_;

    // 运动学接口
    KinematicBaseInterface::SharedPtr kinematic_interface_;

    // 初始化标志
    bool is_initialized_{false};
};
}  // namespace motion_planning_tutorial
#endif
