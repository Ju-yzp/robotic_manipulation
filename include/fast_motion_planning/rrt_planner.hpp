#ifndef FAST_MOTION_PLANNING_RRT_PLANNER_HPP_
#define FAST_MOTION_PLANNING_RRT_PLANNER_HPP_

// fast_motion_planning
#include <fast_motion_planning/NearestNeighbor.hpp>
#include <fast_motion_planning/collision_detector.hpp>
#include <fast_motion_planning/kinematic_interface.hpp>
#include <fast_motion_planning/planner.hpp>
#include <fast_motion_planning/sampler.hpp>
#include <fast_motion_planning/types.hpp>

// cpp
#include <optional>

namespace fast_motion_planning {

class RRTPlanner : virtual public Planner {  // 给UR5E做的规划算法，也可以将思路拓展至冗余机械臂

public:
    explicit RRTPlanner(
        CollisionDetector::SharedPtr& collision_detector,
        KinematicInterface::SharedPtr kinematic_solver, Sampler::UniquePtr& sampler)
        : collision_detector_(collision_detector),
          kinematic_solver_(kinematic_solver),
          sampler_(std::move(sampler)) {
        step_ = 0.12;
        stop_threshold_ = 0.2;
        max_iter_ = 100000;
    }

    RRTPlanner() {}

    void set_step(const double step) { step_ = step; }

    void set_stop_threshold(const double stop_threshold) { stop_threshold_ = stop_threshold; }

    void set_max_iter(uint32_t max_iter) { max_iter_ = max_iter; }

    void plan(PlanningProblem& ppm) override;

private:
    struct StateNode {  // 状态节点
        State state;
        StateNode* parent{nullptr};
    };

    // 检查两个状态之间是否发生碰撞
    std::optional<std::vector<StateNode*>> checkMotion(
        const StateNode* st, const StateNode* dt, uint8_t num);

    // 朝着指定方向扩展
    State expand(const State& st, const State& dt);

    // 获取两个状态之间的欧氏距离
    double distance(const State st, const State dt);

    // 目标附近进行采样
    State goalSample(State goal_state);

    bool checkMotion(State st, State dt);

    // 碰撞检测器
    CollisionDetector::SharedPtr collision_detector_;

    // 运动学解算器
    KinematicInterface::SharedPtr kinematic_solver_;

    struct ImformationStorage {
        StateNode* ptr;
        State point;
    };

    // 固定内存池
    FixedMemoryPool<StateNode, 1000000> fmp_;

    // 邻域搜索
    NearestNeighbor<ImformationStorage, 6, 1000000> nn_;

    // 采样器
    Sampler::UniquePtr sampler_;

    // 随机数生成器
    RandomNumberGenerator rng_;

    // 最大迭代次数
    uint32_t max_iter_;

    // 停止阈值
    double stop_threshold_;

    // 是否把两个状态之间的插值点加入路径中
    bool addIntermediateStates_{false};

    // 步长
    double step_;

    // 目标导向
    double goalBasic_{.08};
};
}  // namespace fast_motion_planning
#endif
