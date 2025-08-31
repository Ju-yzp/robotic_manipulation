#include <motion_planning_tutorial/robot_description.hpp>

#include <iostream>

int main() {
    std::string file_path =
        "/home/up/robotics-manipulation/src/motion_planning_tutorial/config/ur5e_description.yaml";
    namespace mpt = motion_planning_tutorial;
    mpt::RobotDescription rd(file_path, nullptr);
    if (rd.isOverJointPositionLimit(2.5, "elbow")) {
        std::cout << "This joint position is over the limit!" << std::endl;
    }
    return 0;
}
