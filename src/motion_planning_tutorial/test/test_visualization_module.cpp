#include <rclcpp/rclcpp.hpp>
#include <visualization_module/trajectory_visualization.hpp>
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"

class TrajectoryPublisher: public rclcpp::Node {
public:

TrajectoryPublisher(): Node("trajectory_publisher") {
    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_marker", 10);
}
void parseUrdfFile(const std::string urdf_file){
    tv_.loadModel(urdf_file);
}

void update(Eigen::VectorXd state){
    auto marker_array = tv_.getMarkerArray(ns_, 0, alpha_, state);
    publisher_->publish(marker_array);
}
private:
TrajectoryVisualization tv_;

rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
std::string ns_= "world";
double alpha_ = 0.8;
};

int main(int argc, const char *const *argv){
    rclcpp::init(argc,argv);
    TrajectoryPublisher tp;
    tp.parseUrdfFile("/home/up/robotics-manipulation/src/robot_description/urdf/robot.urdf");
    Eigen::VectorXd state(6);
    tp.update(state);
    while(rclcpp::ok()){
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    rclcpp::shutdown();
    return 0;
}