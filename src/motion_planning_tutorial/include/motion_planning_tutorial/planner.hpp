/*
Description: Planner: a class for describing motion planner interface
Author: Jup email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_PLANNER_HPP_
#define MOTION_PLANNING_TOTURIAL_PLANNER_HPP_

// motion_planning_tutorial
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>
#include <motion_planning_tutorial/state.hpp>

namespace motion_planning_tutorial {

class Planner {  // 默认内置使用RRT算法
public:
    void solve(ProblemDefinition& pd);  // 后面引入计时器，设置最大规划时间

private:
    // 机械臂描述
    RobotDescription::SharedPtr robot_description_{nullptr};

    struct Node {
        State state;
        Node* parent;
    };
};

}  // namespace motion_planning_tutorial
#endif  // MOTION_PLANNING_TOTURIAL_PLANNER_HPP_
