#ifndef FAST_MOTION_PLANNING_ROBOT_PARAMS_HPP_
#define FAST_MOTION_PLANNING_ROBOT_PARAMS_HPP_

// cpp
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>

// eigen
#include <eigen3/Eigen/Eigen>

// fast_motion_planning
#include <fast_motion_planning/kinematic_interface.hpp>
#include <fast_motion_planning/types.hpp>

namespace fast_motion_planning {

struct JointLimit {         // 关节约束参数
    double position_upper;  // 关节位置上限
    double position_lower;  // 关节位置下限
    double velocity;        // 速度阈值绝对值
    double acceleration;    // 加速度阈值绝对值
    std::string name;       // 关节名称
};

class RobotParams {
private:
    struct Envelopes {                   // 机械臂包络体
        Eigen::MatrixXd offset;          // 关节下的偏移量
        Eigen::MatrixXd offset_to_base;  // 机械臂基底坐标系下的偏移量
        Eigen::VectorXd radius;          // 球型包络体半径
    };

public:
    REGISTER_SMART_POINTER(RobotParams);

    RobotParams(const std::string config_file, KinematicInterface::SharedPtr kinematic_solver)
        : kinematic_solver_(kinematic_solver) {
        parseConfiguration(config_file);
    }

    RobotParams(const std::string config_file) { parseConfiguration(config_file); }

    RobotParams() = default;

    // 解析配置文件，获取机器人参数
    void parseConfiguration(const std::string config_file);

    // 判断关节位置是否超出限制
    bool isOverJointPositionLimit(const size_t id, double value) noexcept;

    // 判断关节电机加速度是否超出范围
    bool isOverJointVelocity(const size_t id, const double value) noexcept;

    // 设置相关运动学解算器
    void set_kinematic_solver(const KinematicInterface::SharedPtr kinematic_solver) {
        kinematic_solver_ = kinematic_solver;
    }

    // 获取指定索引的关节约束
    std::optional<JointLimit> get_jointlimit(const size_t id) noexcept;

    // 获取指定索引的关节的包络体组
    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> get_envelope_position(
        const size_t id) const;

    // 通过关节名称获取包络体组
    std::optional<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> get_envelope_position(
        const std::string joint_name) const;

    // 电机角度归一化处理,[-π,π]
    static void normalize(double& angle) noexcept;

    // 从指定索引的关节开始更新电机位置，同时更新包络体位置信息,同时其他关节位置是否使用之前值还是使用默认值(0.0)，由is_used_old_value决定
    void update_envelope_position(const State state, size_t start, bool is_use_old_value = true);

    // 返回机械臂自由度
    int get_dof() const { return dof_; }

private:
    // 包络体组
    std::unordered_map<size_t, Envelopes> envelope_group_;

    // 关节约束组
    std::unordered_map<size_t, JointLimit> jointlimit_group_;

    // 是否已被初始化
    bool is_initialized_{false};

    // 机械臂轴数(关节电机数量)
    int dof_{0};

    // 运动学解算器
    KinematicInterface::SharedPtr kinematic_solver_ = nullptr;

    // 电机关节位置旧值
    State old_state_;
};
}  // namespace fast_motion_planning

#endif
