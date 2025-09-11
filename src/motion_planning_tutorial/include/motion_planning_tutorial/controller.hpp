/*
Description: Controller:
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_CONTROLLER_HPP_
#define MOTION_PLANNING_TOTURIAL_CONTROLLER_HPP_

// motion_planning_tutorial
#include <motion_planning_tutorial/non_uniform_bspline.hpp>
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>

namespace motion_planning_tutorial {

enum class SmoothType : uint8_t { BASIC_SPLINE = 0 };

class Controller {
public:
    Controller(const RobotDescription::SharedPtr& robot_description)
        : robot_description_(robot_description) {}

    // 平滑路径:满足c0,c1,c2,c3连续性
    NonUniformBspline smoothPath(
        ProblemDefinition& pd, std::vector<double>& timepoint,
        const SmoothType type = SmoothType::BASIC_SPLINE);

    std::vector<double> set_initial_time_point(const ProblemDefinition& pd);

    void set_id_and_name(const size_t id, const std::string& joint_name) {
        id_map_[id] = joint_name;
    }

private:
    RobotDescription::SharedPtr robot_description_{nullptr};

    // 名称映射表
    std::unordered_map<size_t, std::string> id_map_;
};
}  // namespace motion_planning_tutorial
#endif
