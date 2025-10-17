// fast_motion_planning
#include <fast_motion_planning/rrt_planner.hpp>

// cpp
#include <algorithm>
#include <cstdint>
#include <iostream>

namespace fast_motion_planning {

void RRTPlanner::plan(PlanningProblem& ppm) {
    Eigen::VectorXd st = ppm.get_start_state();

    if (!collision_detector_->isFreeCollision(st)) {
        std::cout << "Invalid start state for robot" << std::endl;
        return;
    }

    std::vector<State> valid_solutions = kinematic_solver_->inverseKinematic(ppm.get_end_state());
    valid_solutions.erase(
        std::remove_if(
            valid_solutions.begin(), valid_solutions.end(),
            [this](const State& state) { return !collision_detector_->isFreeCollision(state); }),
        valid_solutions.end());

    if (valid_solutions.empty()) {
        std::cout << "Failed to find a safe path for the robot to reach the specified pose: no "
                     "valid solutions exist"
                  << std::endl;
        return;
    }

    StateNode* start_node = fmp_.allocate();
    StateNode* goal_node = fmp_.allocate();
    start_node->state = st;
    goal_node->state = valid_solutions[0];
    ImformationStorage is, nearest_node;
    is.ptr = start_node;
    is.point = start_node->state;
    std::vector<ImformationStorage> isv;
    isv.emplace_back(is);
    nn_.build(isv);

    uint32_t count{0};

    State ramdom_state, state;
    StateNode* nearestGoalNode = start_node;
    while (count < max_iter_ && !ppm.get_probelm_state()) {
        if (rng_.uniform01() < goalBasic_)
            ramdom_state = goalSample(valid_solutions[0]);
        else if (rng_.uniform01() < startBaisc_)
            ramdom_state = startSample(st);
        else
            ramdom_state = sampler_->sample();

        // 寻找最邻近点,并且向随机采样点的方向扩展
        nn_.search(ramdom_state, nearest_node);
        state = expand(nearest_node.point, ramdom_state);

        StateNode* node{nullptr};
        if (checkMotion(nearest_node.point, state)) {
            node = fmp_.allocate();
            node->parent = nearest_node.ptr;
            node->state = state;
            is.ptr = node;
            is.point = state;
            count++;
            nn_.add_point(is);
        } else
            continue;

        double dist = distance(node->state, goal_node->state);
        double nearest_dist = dist < distance(nearestGoalNode->state, goal_node->state);
        if (dist < nearest_dist) nearestGoalNode = node;
        if (dist < stop_threshold_) {
            ppm.set_problem_state(true);
            goal_node->parent = node;
            break;
        }
    }

    if (!ppm.get_probelm_state()) return;

    std::vector<State> initial_path, reverse_path;
    StateNode* last_node = goal_node;
    while (last_node) {
        initial_path.emplace_back(last_node->state);
        last_node = last_node->parent;
    }

    reverse_path.reserve(initial_path.size());
    for (auto it = initial_path.rbegin(); it != initial_path.rend(); ++it) {
        reverse_path.emplace_back(*it);
    }

    ppm.set_path(reverse_path);
}

std::optional<std::vector<RRTPlanner::StateNode*>> RRTPlanner::checkMotion(
    const StateNode* st, const StateNode* dt, uint8_t num) {
    std::optional<std::vector<StateNode*>> temp_states;

    std::vector<StateNode*> states;
    states.reserve(num);

    auto dist = dt->state - st->state;
    double step = dist.norm() / double(num + 1);
    State new_state;
    for (int i{1}; i < num + 1; ++i) {
        new_state = st->state + step * double(i) * dist;
        if (!collision_detector_->isFreeCollision(new_state)) break;
        StateNode* node = fmp_.allocate();
        node->state = new_state;
        states.emplace_back(node);
    }
    temp_states = states;
    return temp_states;
}

bool RRTPlanner::checkMotion(State st, State dt) {
    return collision_detector_->isFreeCollision(dt);
}

State RRTPlanner::expand(const State& st, const State& dt) {
    return st + (step_ / (dt - st).norm()) * (dt - st);
}

State RRTPlanner::goalSample(State goal_state) {
    Eigen::Vector<double, 6> positions;
    for (size_t id{0}; id < 6; id++) {
        positions[id] = rng_.uniform(goal_state[id] + step_, goal_state[id] - step_);
    }
    State state{positions};
    return state;
}

State RRTPlanner::startSample(State goal_state) {
    Eigen::Vector<double, 6> positions;
    for (size_t id{0}; id < 6; id++) {
        positions[id] = rng_.uniform(goal_state[id] + 1.6, goal_state[id] - 1.6);
    }
    State state{positions};
    return state;
}

double RRTPlanner::distance(const State st, const State dt) { return (st - dt).norm(); }
}  // namespace fast_motion_planning
