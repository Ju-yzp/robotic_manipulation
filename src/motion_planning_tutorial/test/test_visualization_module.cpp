#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <visualization_module/trajectory_visualization.hpp>

namespace vu = visualization_utils;
class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        publisher_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker", 10);
    }
    void parseUrdfFile(const std::string urdf_file) { tv_.loadModel(urdf_file); }

    void update(std::unordered_map<std::string, double> joint_state_map,int idx) {
        publisher_->publish(
            tv_.getMarkerArray("trajectory_visualization_test", idx, 0.8, joint_state_map));
    }

private:
    vu::TrajectoryVisualization tv_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
};

int main(int argc, const char* const* argv) {
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
    tp.update(joint_state_map,0);

    joint_state_map["base_link-base_link_inertia"] = 1.3;
    joint_state_map["shoulder_pan_joint"] = -0.4;
    joint_state_map["shoulder_lift_joint"] = -0.5;
    joint_state_map["elbow_joint"] = 0.7;
    joint_state_map["wrist_1_joint"] = 0.8;
    joint_state_map["wrist_2_joint"] = 1.0;
    joint_state_map["wrist_3_joint"] = 1.0;
    tp.update(joint_state_map, 1);

    joint_state_map["base_link-base_link_inertia"] = 1.2;
    joint_state_map["shoulder_pan_joint"] = -0.7;
    joint_state_map["shoulder_lift_joint"] = -0.5;
    joint_state_map["elbow_joint"] = 0.7;
    joint_state_map["wrist_1_joint"] = 0.8;
    joint_state_map["wrist_2_joint"] = -1.0;
    joint_state_map["wrist_3_joint"] = 2.1;
    tp.update(joint_state_map, 2);
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    rclcpp::shutdown();

    return 0;
}
