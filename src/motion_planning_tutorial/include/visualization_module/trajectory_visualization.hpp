#ifndef VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_
#define VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_

// cpp
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// urdf
#include <sys/types.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

// tf
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

// eigen
#include <Eigen/Eigen>

#include <rclcpp/clock.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class TrajectoryVisualization {
public:
    struct Link {
        std::string name;
        std::string mesh_file;
        tf2::Transform link_to_fixed =
            tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0f), tf2::Vector3(0.0f, 0.0f, 0.0f));

        tf2::Transform link_offest =
            tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0f), tf2::Vector3(0.0f, 0.0f, 0.0f));
    };

    struct Joint {
        std::string name;
        tf2::Transform tf;
        std::vector<Link> child_links;
    };

    TrajectoryVisualization(const std::string model_description) { loadModel(model_description); }

    TrajectoryVisualization() {}

    // 加载机器人模型
    void loadModel(const std::string& model_description);

    visualization_msgs::msg::MarkerArray getMarkerArray(
        std::string ns, int idx, double alpha, Eigen::VectorXd state);

private:
    // 更新关节位置
    // void updatePositions(const Eigen::VectorXd& state);

    visualization_msgs::msg::Marker getMarker(
        int id, const std::string ns, double alpha, const Eigen::Matrix4d& T,
        const std::string& mesh_file);

    bool hasMeshFile(std::shared_ptr<const urdf::Link>& link, std::string& mesh_file);

    // 是否已经完成初始化
    bool is_initialized_ = false;

    // 命名空间
    std::string ns_;

    // 机械臂连杆
    std::vector<std::shared_ptr<urdf::Link>> links_;

    //
    std::string geometry_type_ = "visual";

    std::map<uint32_t, Link> link_map_;

    std::unordered_map<uint32_t, Joint*> joint_map_;

    rclcpp::Clock clock_;

    std::vector<Link> fixed_links_;
};
#endif  // VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_
