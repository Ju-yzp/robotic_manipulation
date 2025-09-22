// fast_motion_planning
#include <fast_motion_planning/rrt_planner.hpp>

// cpp
#include <algorithm>
#include <cstdint>
#include <iostream>

namespace fast_motion_planning {

// void RRTPlanner::plan(PlanningProblem& ppm) {
//     /*
//     *TODO： 后面会考路引入多线程，也就是如果存在多个有效目标解时，会将其中差异较大的关节构型视为
//     * 目标点，然后如果其中一个树生长成功，则就会通知其他线程，停止工作，若是每个线程工作时间都超过
//      任务规定时间，就会视为任务失败
//      */

//     // 检查机器人初始状态是否合法
//     Eigen::VectorXd st = ppm.get_start_state();

//     if (!collision_detector_->isFreeCollision(st)) {
//         std::cout << "Invalid start state for robot" << std::endl;
//         return;
//     }

//     // 检查是否有使得合法解使得机器人达到目标姿态
//     std::vector<State> valid_solutions =
//     kinematic_solver_->inverseKinematic(ppm.get_end_state()); valid_solutions.erase(
//         std::remove_if(
//             valid_solutions.begin(), valid_solutions.end(),
//             [this](const State& state) { return !collision_detector_->isFreeCollision(state); }),
//         valid_solutions.end());

//     if (valid_solutions.empty()) {
//         std::cout << "Failed to find a safe path for the robot to reach the specified pose: no "
//                      "valid solutions exist"
//                   << std::endl;

//         return;
//     }

//     StateNode* start_node = new StateNode();
//     StateNode* goal_node = new StateNode();
//     start_node->state = st;
//     goal_node->state = valid_solutions[0];
//     nn_.insert(start_node);

//     uint32_t count{0};
//     Eigen::VectorXd arm_state, wrist_state;  // 把手臂部分和腕部分成两个维度
//     Eigen::Vector<double, 6> state;
//     while (count < max_iter_) {
//         count++;
//         arm_state = sampler_->sample(0, 3);
//         if (collision_detector_->isFreeCollision(arm_state))
//             wrist_state = sampler_->sample(3, 3);
//         else
//             continue;
//     }

//     if (!ppm.get_probelm_state()) return;

//     std::vector<State> initial_path, reverse_path;
//     StateNode* last_node = goal_node;
//     while (last_node) {
//         initial_path.emplace_back(last_node->state);
//         last_node = last_node->parent;
//     }

//     reverse_path.reserve(initial_path.size());
//     for (uint32_t id = initial_path.size() - 1; id >= 0; --id)
//         reverse_path.emplace_back(initial_path[id]);

//     // TODO：会添加剪枝步骤
//     ppm.set_path(reverse_path);

//     reset();
// }

// void RRTPlanner::reset() {
//     for (auto node : nodes_)
//         if (node) delete node;

//     root_ = nullptr;
// }

// std::optional<std::vector<RRTPlanner::StateNode*>> RRTPlanner::checkMotion(
//     const StateNode* st, const StateNode* dt, uint8_t num) {
//     std::optional<std::vector<StateNode*>> temp_states;

//     if (num == 0) {
//         std::cout << "" << std::endl;
//         return temp_states;
//     }

//     std::vector<StateNode*> states;
//     states.reserve(num);

//     auto dist = dt->state - st->state;
//     double step = dist.norm() / double(num + 1);
//     State new_state;
//     for (int i{1}; i < num + 1; ++i) {
//         new_state = st->state + step * double(i) * dist;
//         if (!collision_detector_->isFreeCollision(new_state)) break;
//         StateNode* node = new StateNode();
//         node->state = new_state;
//         states.emplace_back(node);
//     }
//     temp_states = states;
//     return temp_states;
// }

// State RRTPlanner::expand(const State& st, const State& dt) {
//     return st + ((dt - st).norm() / step_) * (dt - st);
// }

// double RRTPlanner::distance(const State st, const State dt) { return (st - dt).norm(); }
}  // namespace fast_motion_planning
