/*
Description: Test ur5e correctness by comparing the forward kinematic and inverse kinematic results
Author: Jup email: Jup230551@outlook.com
*/

// cpp
#include <iomanip>
#include <iostream>
#include <memory>

// motion_planning_tutorial
#include <motion_planning_tutorial/kinematic_base_interface.hpp>
#include <motion_planning_tutorial/ur5e_kinematic.hpp>

int main() {
    namespace mpt = motion_planning_tutorial;
    mpt::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    mpt::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    mpt::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    mpt::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::shared_ptr<mpt::Ur5eKInematic> ur5e_kinematic =
        std::make_shared<mpt::Ur5eKInematic>(a_table, d_table, alpha_table, theta_table);

    ur5e_kinematic->set_id(0, "shoulder_pan");
    ur5e_kinematic->set_id(1, "shoulder_lift");
    ur5e_kinematic->set_id(2, "elbow");
    ur5e_kinematic->set_id(3, "wrist_1_link");
    ur5e_kinematic->set_id(4, "wrist_2_link");
    ur5e_kinematic->set_id(5, "wrist_3_link");

    mpt::State state;
    // state.positions.emplace_back(0.2);
    // state.positions.emplace_back(0.4);
    // state.positions.emplace_back(0.5);
    // state.positions.emplace_back(0.6);
    // state.positions.emplace_back(0.7);
    // state.positions.emplace_back(0.8);
    // auto p = ur5e_kinematic->forwardKinematic(state);
    // std::cout << "Isometry3d的4x4矩阵表示：\n"
    //           << std::fixed << std::setprecision(4) << p["elbow"].matrix() << std::endl;

    // auto solutions = ur5e_kinematic->inverseKinematic(p["elbow"]);
    // for (const auto& solution : solutions) {
    //     std::cout << "Inverse Kinematic Solution: ";
    //     for (const auto& pos : solution.positions) {
    //         std::cout << std::fixed << std::setprecision(4) << pos << " ";
    //     }
    //     std::cout << std::endl;
    // }
}
