// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/collision_detector.hpp>
#include <fast_motion_planning/controller.hpp>
#include <fast_motion_planning/non_uniform_bspline.hpp>
#include <fast_motion_planning/planningProblem.hpp>
#include <fast_motion_planning/rrt_planner.hpp>
#include <fast_motion_planning/types.hpp>
#include <fast_motion_planning/ur5e_kinematic.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>

// visualization_module
#include <visualization_tools/trajectory_visualization.hpp>

int main(int argc, const char* const* argv) {
    namespace fmp = fast_motion_planning;
    State start_state = Eigen::Vector<double, 6>{1.0, -0.7, -1.2, 0.2, 0.2, 1.0};
    State end_state = Eigen::Vector<double, 6>{1.30, -0.314, -0.377, -0.503, 0.0, 0.0};

    // UR5E运动学
    fmp::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    fmp::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    fmp::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    fmp::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    fmp::Ur5eKinematicSolver::SharedPtr uks =
        std::make_shared<fmp::Ur5eKinematicSolver>(a_table, d_table, alpha_table, theta_table);

    auto goal_pose = uks->get_endeffector_pose(end_state);

    // 定义规划问题
    fmp::PlanningProblem ppm(start_state, goal_pose);

    // 机器人参数器
    const std::string config_file = "/home/up/robotics-manipulation/config/ur5e_params.yaml";
    fmp::RobotParams::SharedPtr robot_params = std::make_shared<fmp::RobotParams>(config_file);
    robot_params->set_kinematic_solver(uks);

    // 碰撞检测器
    fmp::CollisionDetector::UniquePtr collision_detector =
        std::make_unique<fmp::CollisionDetector>(robot_params);

    // 采样器
    fmp::Sampler::UniquePtr sampler = std::make_unique<fmp::Sampler>(robot_params);

    // 规划器
    fmp::RRTPlanner cp(collision_detector, uks, sampler);

    // 控制器
    fmp::Controller controller(robot_params);
    // 定义障碍物

    cp.plan(ppm);
    if (ppm.get_probelm_state())
        std::cout << "Succeful to solve the planning problem." << std::endl;
    else {
        std::cout << "Failed to solve the planning problem." << std::endl;
        return -1;
    }

    // 平滑路径
    fmp::NonUniformBspline nub = controller.smoothPath(ppm);
}
