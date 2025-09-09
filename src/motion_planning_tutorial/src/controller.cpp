#include <tinyspline.h>
#include <tinysplinecxx.h>
#include <cstddef>
#include <iostream>
#include <limits>
#include <motion_planning_tutorial/controller.hpp>

namespace motion_planning_tutorial {
void Controller::smoothPath(
    ProblemDefinition& pd, std::vector<double>& timepoint, const SmoothType type) {
    const auto path = pd.get_initial_path();
    constexpr double factor = 1.08;  // 缩放因子

    tinyspline::BSpline bs;
    bool is_successful{false};
    // while (!is_successful) {
    tinyspline::BSpline spline(timepoint.size());
    std::vector<tinyspline::real> control_points = spline.controlPoints();

    for (int id{0}; id < path.size(); ++id) {
        control_points[id] = (tsReal)timepoint[id];
        control_points[id + 1] = (tsReal)path[id].positions[0];
    }

    spline.setControlPoints(control_points);
    tinyspline::BSpline velocity = spline.derive().toBeziers();

    std::vector<tinyspline::real> velocity_control_points = velocity.controlPoints();
    for (size_t id{0}; id < velocity_control_points.size() / 2 + 1; ++id) {
        const auto& jointlimit = robot_description_->get_jointlimit(id_map_[0]);
        if (velocity_control_points[id * 2 + 1] > jointlimit.joint_velocity_upper ||
            velocity_control_points[id * 2 + 1] < jointlimit.joint_velocity_lower) {
            std::cout << "!!" << std::endl;
            break;
            //    double diff = (factor - 1.0) * (timepoint[id] - timepoint[id  - 1]);
            //    for(size_t i{id}; i < path.size(); ++i)
            //       timepoint[i] += diff;
            //    break;
        }

        // if(id == path.size() - 1) is_successful = true;
    }
    // bs = spline;
    // }
}

std::vector<double> Controller::set_initial_time_point(const ProblemDefinition& pd) {
    auto path = pd.get_initial_path();

    std::vector<double> timepoint;
    double time{0.0};
    double max_spenttime{0.0};
    timepoint.emplace_back(time);
    for (size_t id{1}; id < path.size() - 1; ++id) {
        max_spenttime = std::numeric_limits<double>::lowest();
        for (const auto& m : id_map_) {
            const auto jointlimit = robot_description_->get_jointlimit(m.second);
            double spenttime = std::abs(
                (path[id + 1].positions(m.first) - path[id].positions(m.first)) /
                jointlimit.joint_velocity_upper);
            max_spenttime = spenttime > max_spenttime ? spenttime : max_spenttime;
        }
        time += max_spenttime;
        timepoint.emplace_back(time);
    }

    return timepoint;
}
}  // namespace motion_planning_tutorial
