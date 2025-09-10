#include <tinyspline.h>
#include <tinysplinecxx.h>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <motion_planning_tutorial/controller.hpp>
#include <thread>

namespace motion_planning_tutorial {
void Controller::smoothPath(
    ProblemDefinition& pd, std::vector<double>& timepoint, const SmoothType type) {
    const auto path = pd.get_initial_path();
    constexpr double factor = 1.60;       // 缩放因子
    constexpr size_t SPLINE_ORDER = 3;    // 样条阶数
    constexpr size_t SAMPLE_COUNT = 100;  // 速度采样点数量
    constexpr double EPS = 1e-6;          // 数值精度

    tinyspline::BSpline bs;
    bool is_successful = false;
    const auto& jointlimit = robot_description_->get_jointlimit(id_map_[0]);

    // 确保路径和时间点数量一致且不为空
    assert(!path.empty() && "Path is empty");
    assert(path.size() == timepoint.size() && "path and timepoint size mismatch");
    const size_t n_points = path.size();

    // 确保时间点单调递增
    for (size_t i = 1; i < n_points; ++i) {
        if (timepoint[i] <= timepoint[i - 1]) {
            timepoint[i] = timepoint[i - 1] + EPS;
        }
    }

    while (!is_successful) {
        // 初始化样条：n_points个控制点，2维（时间+位置），3阶
        tinyspline::BSpline spline(n_points, 2, SPLINE_ORDER);
        std::vector<tinyspline::real> control_points = spline.controlPoints();

        // 填充控制点（2维：[时间, 位置]）
        for (size_t id = 0; id < n_points; ++id) {
            control_points[id * 2] = static_cast<tsReal>(timepoint[id]);  // 时间维度
            control_points[id * 2 + 1] = static_cast<tsReal>(path[id].positions[0]);  // 位置维度
        }
        spline.setControlPoints(control_points);

        // 求导获取参数导数（不是直接速度）
        tinyspline::BSpline deriv;
        try {
            deriv = spline.derive();  // 一阶导数：[dt/du, dp/du]
        } catch (const std::exception& e) {
            std::cerr << "Spline derivation failed: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        // 检查速度约束（使用采样点而非控制点）
        is_successful = true;
        size_t violating_segment = 0;

        // 在样条曲线上均匀采样检查速度
        for (size_t s = 0; s < SAMPLE_COUNT; ++s) {
            const double u = static_cast<double>(s) / (SAMPLE_COUNT - 1);  // [0,1]区间

            // 计算参数u处的导数值
            auto deriv_vals = deriv.eval(u).result();
            const double dt_du = deriv_vals[0];  // 时间对参数的导数
            const double dp_du = deriv_vals[1];  // 位置对参数的导数

            // 避免除零错误
            if (std::abs(dt_du) < EPS) {
                std::cerr << "Time derivative near zero at u=" << u << std::endl;
                is_successful = false;
                violating_segment = s * (n_points - 1) / SAMPLE_COUNT;
                break;
            }

            // 计算实际速度：dp/dt = (dp/du) / (dt/du)
            const double actual_velocity = dp_du / dt_du;

            // 检查速度是否超出限制
            if (actual_velocity > jointlimit.joint_velocity_upper + EPS ||
                actual_velocity < jointlimit.joint_velocity_lower - EPS) {
                std::cout << "Velocity out of limit: " << actual_velocity << " at u=" << u
                          << std::endl;

                is_successful = false;
                // 计算违规点所在的路径段
                violating_segment =
                    std::min(static_cast<size_t>(s * (n_points - 1) / SAMPLE_COUNT), n_points - 2);
                break;
            }
        }

        // 若速度超限，调整时间点（局部等比例拉伸）
        if (!is_successful) {
            // 只调整违规段及后续的时间点
            if (violating_segment + 1 < timepoint.size()) {
                const double original_interval =
                    timepoint[violating_segment + 1] - timepoint[violating_segment];
                const double delta = original_interval * (factor - 1.0);

                // 等比例拉伸后续时间点
                for (size_t i = violating_segment + 1; i < timepoint.size(); ++i) {
                    timepoint[i] += delta;
                }

                std::cout << "Adjusted time segment " << violating_segment << " by delta: " << delta
                          << std::endl;
            }
        } else {
            bs = spline;  // 保存有效的样条
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 缩短等待时间
    }
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
