// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>

// cpp
#include <iostream>

int main() {
    namespace fmp = fast_motion_planning;
    const std::string config_file = "/home/up/robotics-manipulation/config/ur5e_params.yaml";
    fmp::RobotParams robot_params(config_file);

    constexpr int dof = 6;
    std::cout << "-----Test Config Parser------" << std::endl;
    std::cout << std::endl;
    for (int id{0}; id < dof; ++id) {
        std::optional<fmp::JointLimit> jointlimit = robot_params.get_jointlimit(id);
        if (jointlimit.has_value()) {
            std::cout << "Joint name :" << jointlimit->name << std::endl;
            std::cout << "Joint's position upper is :" << jointlimit->position_upper << std::endl;
            std::cout << "Joint's position lower is :" << jointlimit->position_lower << std::endl;
            std::cout << "Joint's velocity limit is :" << jointlimit->velocity << std::endl;
            std::cout << "Joint's aceleration limit is :" << jointlimit->acceleration << std::endl;
            std::cout << std::endl;
        }
    }
}
