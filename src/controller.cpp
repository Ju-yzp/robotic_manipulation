// cpp
#include <cstddef>

// fast_motion_plannning
#include <fast_motion_planning/controller.hpp>

namespace fast_motion_planning {
NonUniformBspline Controller::smoothPath(const PlanningProblem& ppm) {
    const auto initial_path = ppm.get_path();

    const int n = initial_path.size();
    Eigen::MatrixXd control_points(n, 6);

    for (size_t i{0}; i < n; ++i) control_points.row(i) = initial_path[i].topRows(6);

    NonUniformBspline nub(control_points, 3, 0.05, robot_params_);

    Eigen::VectorXd knots(n + 5);
    for (size_t i{0}; i < n + 5; ++i) {
        if (i < 4)
            knots(i) = 0.0;
        else if (i >= 4 && i <= n)
            knots(i) = 0.05 * (i - 3);
        else
            knots(i) = 0.05 * (n - 2);
    }

    nub.set_knot(knots);
    nub.set_has_acceleration_limit(true);

    int max_iter{500}, count{0};
    while (count < max_iter && !nub.checkFeasiblity()) count++;
    return nub;
}
}  // namespace fast_motion_planning
