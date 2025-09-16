// cpp
#include <algorithm>
#include <cstddef>
#include <cstdint>

// motion_planning_tutorial
#include <motion_planning_tutorial/planner.hpp>

namespace motion_planning_tutorial {

void Planner::solve(
    ProblemDefinition& pd) {  // TODO:还没有严格验证，函数内有错误,默认使用UR5E进行测试
    // 判断一开始能不能有满足目标配置
    auto solutions = kinematic_interface_->inverseKinematic(pd.get_goal_state());

    robot_description_->update_envelopes_position(pd.get_start_state());
    for (auto& in : id_map_) {
        const auto jointlimit = robot_description_->get_jointlimit(in.second);
        auto envelopes = robot_description_->get_envelope(in.second);
        for (auto envelope : envelopes) {
            if (collision_detector_->isOccurCollision(
                    envelope.global_translation, envelope.radius)) {
                std::cout << "Invalid start state, collision occurs at joint: " << in.second
                          << std::endl;
                return;  // 有碰撞发生
            }
        }
    }

    solutions.erase(
        std::remove_if(
            solutions.begin(), solutions.end(),
            [this](const auto solution) {
                // 检查是否超出关节限制
                robot_description_->update_envelopes_position(solution);
                for (auto& in : id_map_) {
                    const auto jointlimit = robot_description_->get_jointlimit(in.second);
                    if (solution.positions(in.first) > jointlimit.joint_position_upper ||
                        solution.positions(in.first) < jointlimit.joint_position_lower)
                        return true;
                    auto envelopes = robot_description_->get_envelope(in.second);
                    for (auto envelope : envelopes) {
                        if (collision_detector_->isOccurCollision(
                                envelope.global_translation, envelope.radius)) {
                            return true;  // 有碰撞发生
                        }
                    }
                }
                return false;  // 没有碰撞发生
            }),
        solutions.end());

    Node* goal = new Node();
    Node* start = new Node();

    if (solutions.empty()) {
        std::cout << "Has no invalid goal solution" << std::endl;
        delete goal;
        delete start;
        return;
    } else {
        std::cout << "Has " << solutions.size() << " valid goal solutions" << std::endl;
    }

    goal->state = solutions[0];
    start->state = pd.get_start_state();
    Node* lastest_solution;
    nn_.add(start);  // 将根节点加入

    uint32_t count{0};
    while (count < maxIter_) {
        lastest_solution = new Node();
        if (rng_.uniform01() < goalBasic_)
            lastest_solution->state = goalSample(goal->state);
        else
            lastest_solution->state = uniformSample();

        auto nearest_node = nn_.searchNearestNeighbor(lastest_solution);

        double dist = distance(nearest_node->state, lastest_solution->state);
        if (dist > step_)
            lastest_solution->state = expand(nearest_node->state, lastest_solution->state);

        count++;

        // std::cout << count << std::endl;
        int num{2};

        const auto& st = nearest_node->state;
        const auto& dt = lastest_solution->state;

        if (checkMotion(st, dt, num)) {
            if (addIntermediateStates_) {
                std::vector<State> states = interpolate(st, dt, num, 2);

                Node* last_node = lastest_solution;
                for (size_t start = 1; start < states.size(); ++start) {
                    auto* node = new Node();
                    node->state = states[start];
                    node->parent = last_node;
                    nn_.add(node);
                    last_node = node;
                    lastest_solution = node;
                }
            } else {
                lastest_solution->parent = nearest_node;
                nn_.add(lastest_solution);
            }
        } else {
            delete lastest_solution;
            continue;
        }

        auto node = nn_.searchNearestNeighbor(goal);
        if (distance(node->state, goal->state) < stop_threshold_) {
            pd.update_state(true);
            goal->parent = node;
            std::cout << "Find the path with " << count << " iterations" << std::endl;
            break;
        }
    }

    if (!pd.get_state()) return;

    // 获得原始路径，但是是目标姿态到达起始姿态
    std::vector<State> initial_path;
    while (goal) {
        initial_path.emplace_back(goal->state);
        goal = goal->parent;
    }

    // 翻转路径
    std::vector<State> reverse_path;
    reverse_path.reserve(initial_path.size() + 2);
    for (int id = initial_path.size() - 1; id >= 0; --id)
        reverse_path.emplace_back(initial_path[id]);

    // 保存路径
    pd.set_initial_path(reverse_path);

    nn_.relaseMemoryResource();
}

double Planner::distance(State& st, State& dt) { return (st.positions - dt.positions).norm(); }

State Planner::uniformSample() {
    State state;
    state.positions = Eigen::Vector<double, 6>();  // TODO:暂时先给UR5E作为实验
    for (auto& in : id_map_) {
        auto joint_limit = robot_description_->get_jointlimit(in.second);
        state.positions[in.first] =
            rng_.uniform(joint_limit.joint_position_lower, joint_limit.joint_position_upper);
    }
    return state;
}

State Planner::expand(State& st, State& dt) {
    double dist = distance(st, dt);
    State new_state;
    new_state.positions = st.positions + (dt.positions - st.positions) * (step_ / dist);
    return new_state;
}

State Planner::goalSample(State goal_state) {
    Eigen::Vector<double, 6> positions;
    for (size_t id{0}; id < 6; id++) {
        positions[id] =
            rng_.uniform(goal_state.positions[id] - step_, goal_state.positions[id] + step_);
    }
    State state{positions};
    return state;
}

bool Planner::checkMotion(
    State st, State dt, int& num) {  // 对状态之间进行插值，然后检查中途是否发生碰撞
    std::vector<State> states;

    states.resize(num + 2);
    states[0] = st;
    states[num + 1] = dt;
    for (int i{1}; i <= num; ++i) {
        states[i].positions =
            st.positions + (dt.positions - st.positions) * (double(i) / double(num + 1));
    }

    int count{0};
    for (const auto& state : states) {
        robot_description_->update_envelopes_position(state);
        for (auto& in : id_map_) {
            auto envelopes = robot_description_->get_envelope(in.second);
            for (auto envelope : envelopes) {
                if (collision_detector_->isOccurCollision(
                        envelope.global_translation, envelope.radius)) {
                    num = count;
                    return false;  // 有碰撞发生
                }
            }
        }
        count++;
    }
    return true;
}

std::vector<State> Planner::interpolate(
    const State& st, const State& dt, const int num, const int in) {
    std::vector<State> states;
    if (num == in) {
        // std::cout << "wow" << std::endl;
        states.resize(num + 2);
        states[num + 1] = dt;
    } else
        states.resize(num + 1);

    states[0] = st;

    for (int i{1}; i < num + 1; ++i)
        states[i].positions =
            st.positions + (dt.positions - st.positions) * (double(i) / double(in));

    return states;
}
}  // namespace motion_planning_tutorial
