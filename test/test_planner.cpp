// fast_motion_planning
#include <chrono>
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
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_tools/trajectory_visualization.hpp>

// visualization_module
#include <unordered_map>

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher(const std::string model_description) : Node("visualization_node") {
        tv_.loadModel(model_description);

        pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "vusaliation_trajectory", 1);
    }

    // 发布轨迹
    void publish_trajectory(const std::unordered_map<std::string, double>& jsp) {
        visualization_msgs::msg::MarkerArray tarjectory;
        tarjectory = tv_.getMarkerArray(ns_, idx_, 0.8, jsp);
        pub_->publish(tarjectory);
        idx_++;
    }

    // 发布障碍物信息
    void publish_obstacles(const Eigen::MatrixXd obsp, const Eigen::VectorXd obrs) {}

private:
    visualization_tools::TrajectoryVisualization tv_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;

    int idx_{0};

    std::string ns_{"trajectory_visualization"};
};

int main(int argc, const char* const* argv) {
    namespace fmp = fast_motion_planning;
    State start_state = Eigen::Vector<double, 6>{1.0, -0.7, -1.2, 0.2, 0.2, 1.0};
    State end_state = Eigen::Vector<double, 6>{-1.30, -0.314, -0.377, -0.503, 0.0, 0.0};

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
    const std::string config_file = "/home/up/robotic-manipulation/config/ur5e_params.yaml";
    fmp::RobotParams::SharedPtr robot_params = std::make_shared<fmp::RobotParams>(config_file);
    robot_params->set_kinematic_solver(uks);

    // 碰撞检测器
    fmp::CollisionDetector::SharedPtr collision_detector =
        std::make_shared<fmp::CollisionDetector>(robot_params);

    // 采样器
    fmp::Sampler::UniquePtr sampler = std::make_unique<fmp::Sampler>(robot_params);

    // 规划器
    fmp::RRTPlanner cp(collision_detector, uks, sampler);

    // 控制器
    fmp::Controller controller(collision_detector);

    // 定义障碍物
    Eigen::MatrixXd obstacles_position;
    Eigen::Matrix<double, 4, 3> ospn;
    ospn << -200.0, 500.0, 600.0, 1.0, 100.0, -200.0, 650.0, 1.0, 300.0, 400.0, 400.0, 1.0;
    obstacles_position = ospn;
    Eigen::VectorXd obstacles_radius;
    Eigen::Vector<double, 3> osrs;
    osrs << 100.0, 100.0, 100.0;
    collision_detector->set_obstacles(ospn, osrs);

    // 进行规划
    cp.plan(ppm);
    if (ppm.get_probelm_state())
        std::cout << "Succeful to solve the planning problem." << std::endl;
    else {
        std::cout << "Failed to solve the planning problem." << std::endl;
        return -1;
    }

    // 平滑路径
    fmp::NonUniformBspline nub = controller.smoothPath(ppm);

    // 轨迹可行性检查
    if (controller.checkAllTrajectory(nub)) {
        std::cout << "Optimized trajectory is safe" << std::endl;
    } else {
        std::cout << "Optimized tarjectory is unsafe " << std::endl;
        return -1;
    }
    rclcpp::init(argc, argv);

    std::string model_description = "/home/up/robotic-manipulation/robot_resource/urdf/ur5e.urdf";
    TrajectoryPublisher tp(model_description);

    int num = 10;
    double step = nub.getTimeSum() / double(num);
    for (int i{0}; i < num; ++i) {
        auto state = nub.evaluateDeBoorT(double(i) * step);
        std::unordered_map<std::string, double> jsp;
        jsp["base_link-base_link_inertia"] = 0.0;
        jsp["shoulder_pan_joint"] = state[0];
        jsp["shoulder_lift_joint"] = state[1];
        jsp["elbow_joint"] = state[2];
        jsp["wrist_1_joint"] = state[3];
        jsp["wrist_2_joint"] = -state[4];
        jsp["wrist_3_joint"] = state[5];
        tp.publish_trajectory(jsp);
    }
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    rclcpp::shutdown();
    return 0;
}
