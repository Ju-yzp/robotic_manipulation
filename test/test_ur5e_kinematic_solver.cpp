// fast_motion_planning
#include <fast_motion_planning/ur5e_kinematic.hpp>

// cpp
#include <iostream>

int main() {
    namespace fmp = fast_motion_planning;
    fmp::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    fmp::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    fmp::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    fmp::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    fmp::Ur5eKinematicSolver::SharedPtr ur5e_kinematic_solver =
        std::make_shared<fmp::Ur5eKinematicSolver>(a_table, d_table, alpha_table, theta_table);

    State state = Eigen::Vector<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const auto tf_map = ur5e_kinematic_solver->forwardKinematic(state);

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    for (const auto tf : tf_map) {
        std::cout << std::endl;
        std::cout << tf.second.format(CleanFmt) << std::endl;
    }
}
