/*
Description: ProblemDefinition: a class for defining the problem of motion planning, including start
state, goal state, and initial path Author: Jup email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_PROBLEM_DEFINITION_HPP_
#define MOTION_PLANNING_TOTURIAL_PROBLEM_DEFINITION_HPP_

// motion_planning_tutorial
#include <motion_planning_tutorial/state.hpp>

// eigen
#include <Eigen/Eigen>

// cpp
#include <vector>

namespace motion_planning_tutorial {
class ProblemDefinition {
public:
    ProblemDefinition(State start_state, Eigen::Isometry3d goal_state)
        : start_state_(start_state), goal_state_(goal_state) {}

    ProblemDefinition() = delete;

    ProblemDefinition(ProblemDefinition&) = delete;

    ~ProblemDefinition() = default;

    // 更新状态
    void update_state(bool is_solved) { is_solved_ = is_solved; }

    // 获取起始状态
    State get_start_state() const { return start_state_; }

    // 获取目标状态
    Eigen::Isometry3d get_goal_state() const { return goal_state_; }

    // 设置初始路径
    void set_initial_path(const std::vector<State>& initial_path) { initial_path_ = initial_path; }

    // 获取状态
    const bool get_state() { return is_solved_; }

private:
    State start_state_;  //  机械臂起始状态

    Eigen::Isometry3d goal_state_;  // 机械臂目标状态，也就是末端执行器的目标位姿

    std::vector<State> initial_path_ = std::vector<State>();  // 初始路径,没有经过平滑和约束处理

    bool is_solved_{false};  // 问题是否已经被解决，默认没有被解决
};
}  // namespace motion_planning_tutorial

#endif
