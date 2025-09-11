// motion_planning_tutorial
#include <motion_planning_tutorial/collision_detector.hpp>
#include <motion_planning_tutorial/controller.hpp>
#include <motion_planning_tutorial/planner.hpp>
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>
#include <motion_planning_tutorial/ur5e_kinematic.hpp>
// cpp
#include <iostream>

int main() {
    namespace mpt = motion_planning_tutorial;
    // 先定义规划问题
    Eigen::Vector<double, 6> start_positions;
    start_positions << 0.8, -0.4, -0.2, 1.0, 1.0, -2.0;
    mpt::State start_state;
    start_state.positions = start_positions;
    Eigen::Isometry3d goal_state = Eigen::Isometry3d::Identity();
    goal_state.translation() << 200.0, -300.0, 300.0;
    mpt::ProblemDefinition pd(start_state, goal_state);

    // UR5E运动学
    mpt::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    mpt::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    mpt::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    mpt::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::shared_ptr<mpt::Ur5eKinematic> ur5e_kinematic =
        std::make_shared<mpt::Ur5eKinematic>(a_table, d_table, alpha_table, theta_table);

    ur5e_kinematic->set_id(0, "shoulder_pan");
    ur5e_kinematic->set_id(1, "shoulder_lift");
    ur5e_kinematic->set_id(2, "elbow");
    ur5e_kinematic->set_id(3, "wrist_1_link");
    ur5e_kinematic->set_id(4, "wrist_2_link");
    ur5e_kinematic->set_id(5, "wrist_3_link");

    // 机械臂描述
    std::string configuration_file =
        "/home/up/robotics-manipulation/src/motion_planning_tutorial/config/ur5e_description.yaml";
    mpt::RobotDescription::SharedPtr robot_description =
        std::make_shared<mpt::RobotDescription>(configuration_file, ur5e_kinematic);

    // 碰撞检测器
    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;
    kd_tree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>();
    mpt::CollisionDetector::UniquePtr collision_detector =
        std::make_unique<mpt::CollisionDetector>(kd_tree_);

    // 规划器
    mpt::Planner planner(robot_description, ur5e_kinematic, collision_detector);
    planner.set_id_and_name(0, "shoulder_pan");
    planner.set_id_and_name(1, "shoulder_lift");
    planner.set_id_and_name(2, "elbow");
    planner.set_id_and_name(3, "wrist_1_link");
    planner.set_id_and_name(4, "wrist_2_link");
    planner.set_id_and_name(5, "wrist_3_link");

    planner.solve(pd);

    if (pd.get_state()) std::cout << "Succeful to solve the planning problem." << std::endl;

    for (const auto& state : pd.get_initial_path())
        std::cout << state.positions.transpose() << std::endl;

    mpt::Controller controller(robot_description);
    controller.set_id_and_name(0, "shoulder_pan");
    controller.set_id_and_name(1, "shoulder_lift");
    controller.set_id_and_name(2, "elbow");
    controller.set_id_and_name(3, "wrist_1_link");
    controller.set_id_and_name(4, "wrist_2_link");
    controller.set_id_and_name(5, "wrist_3_link");

    auto timepoint = controller.set_initial_time_point(pd);
    controller.smoothPath(pd, timepoint, mpt::SmoothType::BASIC_SPLINE);
    // std::cout<<"Size is" <<(int)timepoint.size()<<std::endl;
    // for(const auto time:timepoint)
    //     std::cout<<time<<std::endl;
}
