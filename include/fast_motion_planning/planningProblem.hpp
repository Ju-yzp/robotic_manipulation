#ifndef FAST_MOTION_PLANNING_PLANNING_PROBLEM_HPP_
#define FAST_MOTION_PLANNING_PLANNING_PROBLEM_HPP_

#include <Eigen/Eigen>

#include <fast_motion_planning/types.hpp>

namespace fast_motion_planning {

class PlanningProblem {  // 规划问题
public:
    PlanningProblem(const State& start_state, const Eigen::MatrixXd& end_state)
        : start_state_(start_state), end_state_(end_state) {}

    PlanningProblem() = delete;

    // 获取规划问题当前状态
    bool get_probelm_state() const { return is_solved_; }

    // 获取机器人初始状态
    State get_start_state() const { return start_state_; }

    // 获取机器人末端执行器目标姿态
    Eigen::Matrix4d get_end_state() { return end_state_; }

    // 获取初始规划路径(没有动力学约束和平滑处理)
    std::vector<Eigen::VectorXd> get_path() const { return path_; }

    // 设置初始路径
    void set_path(const std::vector<State>& path) { path_ = path; }

private:
    Eigen::VectorXd start_state_;  // 机器人初始状态

    Eigen::Matrix4d end_state_;  // 机械臂末端执行器目标姿态

    std::vector<State> path_;  // 规划出的初始路径

    bool is_solved_{false};  // 规划问题解决标志
};
}  // namespace fast_motion_planning

#endif
