// motion_planning_tutorial
#include <motion_planning_tutorial/collision_detector.hpp>
#include <motion_planning_tutorial/controller.hpp>
#include <motion_planning_tutorial/planner.hpp>
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>
#include <motion_planning_tutorial/ur5e_kinematic.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>

// visualization_module
#include <visualization_module/trajectory_visualization.hpp>

namespace vu = visualization_utils;
class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        publisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker", 10);
    }
    void parseUrdfFile(const std::string urdf_file) { tv_.loadModel(urdf_file); }

    void update(std::unordered_map<std::string, double> joint_state_map, int idx) {
        publisher_->publish(
            tv_.getMarkerArray("trajectory_visualization_test", idx, 0.6, joint_state_map));
    }

private:
    vu::TrajectoryVisualization tv_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

int main(int argc, const char* const* argv) {
    namespace mpt = motion_planning_tutorial;

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

    // 先定义规划问题
    Eigen::Vector<double, 6> start_positions;
    start_positions << 1.30, -0.314, -0.377, -0.503, 0.0, 0.0;
    mpt::State start_state;
    start_state.positions = start_positions;
    mpt::State end_state;
    Eigen::Vector<double, 6> end_positions;
    end_positions << -2.32, 0.203, -0.523, 0.0, 0.0, 0.0;
    end_state.positions = end_positions;
    Eigen::Isometry3d goal_state = ur5e_kinematic->get_endeffector_pose(end_state);
    mpt::ProblemDefinition pd(start_state, goal_state);

    // 机械臂描述
    std::string configuration_file =
        "/home/up/robotics-manipulation/src/motion_planning_tutorial/config/ur5e_description.yaml";
    mpt::RobotDescription::SharedPtr robot_description =
        std::make_shared<mpt::RobotDescription>(configuration_file, ur5e_kinematic);

    // 碰撞检测器
    mpt::Scene scene;
    scene.obstacle_centers = {
        Eigen::Vector4d(-200.0, 500.0, 600.0, 1.0), Eigen::Vector4d(100.0, -200.0, 650.0, 1.0),
        Eigen::Vector4d(300.0, 400.0, 400.0, 1.0), Eigen::Vector4d(300.0, -100.0, 200.0, 1.0)};
    scene.obstacle_radius = {100.0, 100.0, 100.0, 100.0};
    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;
    kd_tree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>();
    mpt::CollisionDetector::UniquePtr collision_detector =
        std::make_unique<mpt::CollisionDetector>(kd_tree_);

    collision_detector->set_scene(scene);

    // 规划器
    mpt::Planner planner(robot_description, ur5e_kinematic, collision_detector);
    planner.set_id_and_name(0, "shoulder_pan");
    planner.set_id_and_name(1, "shoulder_lift");
    planner.set_id_and_name(2, "elbow");
    planner.set_id_and_name(3, "wrist_1_link");
    planner.set_id_and_name(4, "wrist_2_link");
    planner.set_id_and_name(5, "wrist_3_link");

    planner.solve(pd);

    if (pd.get_state())
        std::cout << "Succeful to solve the planning problem." << std::endl;
    else {
        std::cout << "Failed to solve the planning problem." << std::endl;
        return -1;
    }

    auto path = pd.get_initial_path();
    for (const auto& state : path) {
        std::cout << "State: ";
        for (const auto& pos : state.positions) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }

    mpt::Controller controller(robot_description);
    controller.set_id_and_name(0, "shoulder_pan");
    controller.set_id_and_name(1, "shoulder_lift");
    controller.set_id_and_name(2, "elbow");
    controller.set_id_and_name(3, "wrist_1_link");
    controller.set_id_and_name(4, "wrist_2_link");
    controller.set_id_and_name(5, "wrist_3_link");

    auto timepoint = controller.set_initial_time_point(pd);
    auto bspilne = controller.smoothPath(pd, timepoint, mpt::SmoothType::BASIC_SPLINE);

    double time_sum = bspilne.getTimeSum();
    double time_step = time_sum / double(7);

    rclcpp::init(argc, argv);
    TrajectoryPublisher tp;
    std::string urdf_file = "/home/up/robotics-manipulation/src/robot_description/urdf/ur5e.urdf";
    tp.parseUrdfFile(urdf_file);
    std::unordered_map<std::string, double> joint_state_map;
    joint_state_map["base_link-base_link_inertia"] = 0.0;
    joint_state_map["shoulder_pan_joint"] = 0.0;
    joint_state_map["shoulder_lift_joint"] = 0.0;
    joint_state_map["elbow_joint"] = 0.0;
    joint_state_map["wrist_1_joint"] = 0.0;
    joint_state_map["wrist_2_joint"] = 0.0;
    joint_state_map["wrist_3_joint"] = 0.0;

    for (uint8_t i{0}; i < 7; ++i) {
        auto positions = bspilne.evaluateDeBoor(time_step * (double)i);
        std::unordered_map<std::string, double> joint_state_map;
        joint_state_map["base_link-base_link_inertia"] = 0.0;
        joint_state_map["shoulder_pan_joint"] = positions(0);
        joint_state_map["shoulder_lift_joint"] = positions(1);
        joint_state_map["elbow_joint"] = positions(2);
        joint_state_map["wrist_1_joint"] = positions(3);
        joint_state_map["wrist_2_joint"] = -positions(4);
        joint_state_map["wrist_3_joint"] = positions(5);
        tp.update(joint_state_map, i);
    }
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    rclcpp::shutdown();
    return 0;
}
