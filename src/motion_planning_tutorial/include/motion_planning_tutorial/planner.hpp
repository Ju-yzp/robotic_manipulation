/*
Description: Planner: a class for describing motion planner interface
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_PLANNER_HPP_
#define MOTION_PLANNING_TOTURIAL_PLANNER_HPP_

// motion_planning_tutorial
#include <motion_planning_tutorial/NearestNeighbors.hpp>
#include <motion_planning_tutorial/RandomNumberGenerator.hpp>
#include <motion_planning_tutorial/collision_detector.hpp>
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>
#include <motion_planning_tutorial/state.hpp>

// cpp
#include <unordered_map>

namespace motion_planning_tutorial {

class Planner {  // 默认内置使用RRT算法
public:
    Planner(
        RobotDescription::SharedPtr robot_description,
        KinematicBaseInterface::SharedPtr kinematic_interface,
        CollisionDetector::UniquePtr& collision_detector, double step = 0.05,
        double stop_threshold = 0.06)
        : robot_description_(robot_description),
          kinematic_interface_(kinematic_interface),
          collision_detector_(std::move(collision_detector)),
          step(step),
          stop_threshold_(stop_threshold) {}

    void solve(ProblemDefinition& pd);  // 后面引入计时器，设置最大规划时间

    void set_id_and_name(const size_t id, const std::string name) { id_map_[id] = name; }

private:
    State uniformSample();

    State goalSample(State goal_state);

    double distance(State& st, State& dt);

    State expand(State& st, State& dt);

    // 检查两个状态是否发生碰撞
    bool checkMotion(State st, State dt);

    struct Node {
        State state;
        Node* parent{nullptr};
    };

    // 机械臂描述
    RobotDescription::SharedPtr robot_description_{nullptr};

    uint32_t maxIter_{100000};

    double goalBasic_{.05};

    // 随机数字生成器
    RangeNumberGenerator rng_;

    // 运动学接口
    KinematicBaseInterface::SharedPtr kinematic_interface_;

    // 邻近搜索
    NearestNeighbors<Node> nn_;

    // 名称映射表
    std::unordered_map<size_t, std::string> id_map_;

    // 碰撞检测器
    CollisionDetector::UniquePtr collision_detector_;

    // 步长
    double step;

    // 停止的阈值
    double stop_threshold_;

    // 是否把两个状态之间的采样点加入路径中
    bool addIntermediateStates_{false};
};

}  // namespace motion_planning_tutorial
#endif  // MOTION_PLANNING_TOTURIAL_PLANNER_HPP_
