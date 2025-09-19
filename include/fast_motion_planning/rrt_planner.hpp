#ifndef FAST_MOTION_PLANNING_RRT_PLANNER_HPP_
#define FAST_MOTION_PLANNING_RRT_PLANNER_HPP_

// fast_motion_planning
#include <fast_motion_planning/NearestNeighbors.hpp>
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
    void plan(PlanningProblem& ppm) override;

private:
    struct StateNode {  // 状态节点
        State state;
        StateNode* parent{nullptr};
    };

    // 清除数据
    void reset();

    // 检查两个状态之间是否发生碰撞
    std::optional<std::vector<StateNode*>> checkMotion(
        const StateNode* st, const StateNode* dt, uint8_t num);

    // 朝着指定方向扩展
    State expand(const State& st, const State& dt);

    // 获取两个状态之间的欧氏距离
    double distance(const State st, const State dt);

    // 碰撞检测器
    CollisionDetector::SharedPtr collision_detector_;

    // 运动学解算器
    KinematicInterface::SharedPtr kinematic_solver_;

    // 邻域搜索
    NearestNeighbors<StateNode> nn_;

    // 采样器
    Sampler::UniquePtr sampler_;

    // 根节点
    StateNode* root_;

    // 随机生长树节点
    std::vector<StateNode*> nodes_;

    // 最大迭代次数
    uint32_t max_iter_;

    // 停止阈值
    double stop_threshold_;

    // 是否把两个状态之间的插值点加入路径中
    bool addIntermediateStates_{false};

    // 步长
    double step_;
};
}  // namespace fast_motion_planning
#endif
