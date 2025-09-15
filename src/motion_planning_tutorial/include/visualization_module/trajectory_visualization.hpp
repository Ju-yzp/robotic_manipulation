#ifndef VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_
#define VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_

// cpp
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// urdf
#include <sys/types.h>
#include <urdf/model.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

// tf
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

// eigen
#include <Eigen/Eigen>

// rclcpp
#include <rclcpp/clock.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization_utils {

using namespace std;

class TrajectoryVisualization {
public:
    struct Link {
        Eigen::Matrix4d offest{Eigen::Matrix4d::Identity()};         // 姿态
        Eigen::Matrix4d pose_to_fixed{Eigen::Matrix4d::Identity()};  // 相对于基坐标的位姿
        string mesh_file{""};                                        // 纹理文件
        string name;                                                 // 连杆名称
    };

    enum class JointRotationAxis : uint8_t { X = 0, Y = 1, Z = 2 };

    struct Joint {
        Eigen::Matrix4d origin_pose{Eigen::Matrix4d::Identity()};  // 关节原始位姿
        Eigen::Matrix4d new_pose{Eigen::Matrix4d::Identity()};     // 变换后的新姿态
        Link child_link;                                           // 子连杆
        string name;                                               // 关节名称
        vector<Joint*> child_joints;                               // 子关节

        bool is_fixed{true};    // 固定关节标志
        JointRotationAxis jra;  // 关节旋转类型

        void update_state(const double angle) {
            new_pose = origin_pose * get_tranform_matrix(angle);
        }

    private:
        Eigen::Matrix4d get_tranform_matrix(const double angle) {
            Eigen::Matrix4d transform_matrix;
            if (jra == JointRotationAxis::X)
                transform_matrix << 1.0, 0.0, 0.0, 0.0, 0.0, cos(angle), -sin(angle), 0.0, 0.0,
                    sin(angle), cos(angle), 0.0, 0.0, 0.0, 0.0, 1.0;
            else if (jra == JointRotationAxis::Y)
                transform_matrix << cos(angle), 0.0, sin(angle), 0.0, 0.0, 1.0, 0.0, 0.0,
                    -sin(angle), 0.0, cos(angle), 0.0, 0.0, 0.0, 0.0, 1.0;
            else
                transform_matrix << cos(angle), -sin(angle), 0.0, 0.0, sin(angle), cos(angle), 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            return transform_matrix;
        }
    };

    TrajectoryVisualization() = default;

    TrajectoryVisualization(const string& model_description) { loadModel(model_description); }

    ~TrajectoryVisualization() {
        for (auto& joint : joints_) {
            if (joint) delete joint;
        }

        joints_.clear();
        root_joints_.clear();
    }

    // 解析urdf文件
    void loadModel(const string& model_description);

    //
    visualization_msgs::msg::MarkerArray getMarkerArray(
        string ns, int idx, double alpha,
        const std::unordered_map<std::string, double>& joint_state_pair);

private:
    // 更新关节状态
    void updateState(
        const std::unordered_map<std::string, double>& joint_state_pair, Joint* joint,
        Eigen::Matrix4d& tf);

    // 获取marker,所有产生的marker的frame_id默认使用"map"
    visualization_msgs::msg::Marker getMarker(
        int id, const string ns, double alpha, const Eigen::Matrix4d& T, const string& mesh_file);

    // 处理连杆，递归处理urdf文件，使用树状结构描述
    void processJoint(
        shared_ptr<urdf::Joint>& joint, const urdf::Model& model, Joint* parent_joint = nullptr,
        bool is_root = false);

    // 获取连杆的纹理文件信息
    bool get_meshfile(shared_ptr<const urdf::Link>& link, string& mesh_file);

    void get_link_pose(Link& link, shared_ptr<const urdf::Link>& urdf_link);

    bool is_initialized_ = false;  // 是否初始化完成

    string geometry_type_ = "visual";  // 几何类型

    vector<Joint*> joints_;  // 关节

    vector<Joint*> root_joints_;  // 基座关节

    rclcpp::Clock clock_;  // ros2系统时钟（给rviz2使用的）
};
}  // namespace visualization_utils
#endif  // VISUALIZATION_MODULE_TRAJECTORY_VISUALIZATION_HPP_
