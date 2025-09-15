// cpp
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>

// motion_planning_tutorial
#include <motion_planning_tutorial/controller.hpp>

namespace motion_planning_tutorial {
NonUniformBspline Controller::smoothPath(
    ProblemDefinition& pd, std::vector<double>& timepoint, const SmoothType type) {
    const auto path = pd.get_initial_path();
    std::cout << "Path length is " << int(path.size()) << std::endl;

    const int n = path.size();
    Eigen::MatrixXd control_points(n, 6);

    for (int i = 0; i < n; ++i) {
        if (path[i].positions.rows() < 6) {
            std::cerr << "Error: positions has less than 6 rows at index " << i << std::endl;
            control_points.row(i).setZero();
            continue;
        }
        control_points.row(i) = path[i].positions.topRows(6);
    }

    NonUniformBspline non_uniform_bspline(control_points, 3, 0.05, robot_description_, id_map_);

    // TODO:时间控制点粗略估计，其实很容易造成电机性能浪费,后面会改进
    Eigen::VectorXd knot(n + 5);
    for (size_t i{0}; i < n + 5; ++i) {
        if (i < 4)
            knot(i) = 0.0;
        else if (i >= 4 && i <= n)
            knot(i) = 0.05 * (i - 3);
        else
            knot(i) = 0.05 * (n - 2);
    }

    non_uniform_bspline.set_knot(knot);
    non_uniform_bspline.set_has_acceleration_limit(true);

    while (!non_uniform_bspline.checkFeasiblity() && !non_uniform_bspline.reallocateTime()) {
        double timesum = non_uniform_bspline.getTimeSum();
        std::cout << "Time sum: " << timesum << std::endl;
    }
    double timesum = non_uniform_bspline.getTimeSum();
    std::cout << "Time sum: " << timesum << std::endl;

    // 采样，然后写入文件
    int max_sample_count = 16000;
    double time_step = timesum / double(max_sample_count);
    std::ofstream outFile(
        "/home/up/motion_planning/python_tool/statistic_data.txt", std::ios::trunc);
    for (int i{0}; i < max_sample_count; i++) {
        Eigen::VectorXd sample_point = non_uniform_bspline.evaluateDeBoor(time_step * i);
        std::ofstream outFile(
            "/home/up/motion_planning/python_tool/statistic_data.txt", std::ios::app);
        if (outFile.is_open()) {
            outFile << i * time_step << " " << sample_point(0) << " " << sample_point(1) << " "
                    << sample_point(2) << " " << sample_point(3) << " " << sample_point(4) << " "
                    << sample_point(5) << std::endl;
            outFile.close();
        }
    }
    return non_uniform_bspline;
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
                jointlimit.joint_velocity);
            max_spenttime = spenttime > max_spenttime ? spenttime : max_spenttime;
        }
        time += max_spenttime;
        timepoint.emplace_back(time);
    }

    return timepoint;
}
}  // namespace motion_planning_tutorial
