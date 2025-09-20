// fast_motion_planning
#include <fast_motion_planning/sampler.hpp>
#include <fast_motion_planning/ur5e_kinematic.hpp>

// cpp
#include <cstddef>
#include <iostream>
#include <memory>

int main() {
    namespace fmp = fast_motion_planning;
    // 机器人参数
    const std::string config_file = "/home/up/robotics-manipulation/config/ur5e_params.yaml";
    fmp::RobotParams::SharedPtr robot_params = std::make_shared<fmp::RobotParams>(config_file);

    // 采样器
    fmp::Sampler sampler(robot_params);

    State state = sampler.sample(3, 3);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << state.format(CleanFmt) << std::endl;

    // ur5e运动学解算器
    fmp::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    fmp::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    fmp::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    fmp::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    fmp::Ur5eKinematicSolver::SharedPtr ur5e_kinematic_solver =
        std::make_shared<fmp::Ur5eKinematicSolver>(a_table, d_table, alpha_table, theta_table);

    robot_params->set_kinematic_solver(ur5e_kinematic_solver);
    robot_params->update_envelope_position(state, 3, false);

    const size_t dim = 6;

    for (size_t i{0}; i < dim; ++i) {
        auto envelopes_info = robot_params->get_envelope_position(i);

        if (envelopes_info.has_value()) {
            std::cout << std::endl;
            std::cout << "------POSITION-------" << std::endl;
            std::cout << envelopes_info->first.format(CleanFmt) << std::endl;
            std::cout << "------RADIUS-------" << std::endl;
            std::cout << envelopes_info->second.format(CleanFmt) << std::endl;
        }
    }
}
