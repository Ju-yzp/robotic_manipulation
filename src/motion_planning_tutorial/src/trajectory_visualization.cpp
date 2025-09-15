// visualization_module
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <visualization_module/trajectory_visualization.hpp>

// urdf
#include <Eigen/src/Geometry/Quaternion.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

// cpp
#include <iostream>
#include <memory>
#include "tf2/LinearMath/Transform.hpp"
#include <visualization_msgs/msg/detail/marker__struct.hpp>

// ros2
#include <rclcpp/rclcpp.hpp>

using namespace std;

void TrajectoryVisualization::loadModel(const std::string& model_description) {
    if (is_initialized_) return;

    urdf::Model model;

    if (model.initFile(model_description)) {
        auto root_link = model.getRoot();
        if (!root_link) return;

        Link link;
        link.name = root_link->name;
        // fixed_links.emplace_back(link);
        std::vector<urdf::JointSharedPtr> child_joints = root_link->child_joints;

        tf2::Transform tf =
            tf2::Transform(tf2::Quaternion(0.0, 0.0, 0.0, 1.0f), tf2::Vector3(0.0f, 0.0f, 0.0f));

        // 基坐标下的固定连杆，位姿不会随着关节的转动发生变化
        while (child_joints.size() == 1) {
            auto& joint = child_joints[0];
            auto pose = joint->parent_to_joint_origin_transform;
            tf2::Transform temp_tf = tf2::Transform(
                tf2::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
                tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
            tf *= temp_tf;
            if (joint->type == urdf::Joint::FIXED) {
                auto child_link = model.getLink(joint->child_link_name);
                link.name = child_link->name;
                link.link_to_fixed = tf;
                if (child_link->visual) {
                    auto link_to_joint = child_link->visual->origin;
                    link.link_offest = tf2::Transform(
                        tf2::Quaternion(
                            link_to_joint.rotation.x, link_to_joint.rotation.y,
                            link_to_joint.rotation.z, link_to_joint.rotation.w),
                        tf2::Vector3(
                            link_to_joint.position.x, link_to_joint.position.y,
                            link_to_joint.position.z));

                    if (hasMeshFile(child_link, link.mesh_file)) fixed_links_.emplace_back(link);
                }
                child_joints = child_link->child_joints;
            } else
                break;
        }

        std::vector<Link> links;
        Joint* last_joint = nullptr;
        uint32_t id{0};
        function<void(vector<urdf::JointSharedPtr>&)> func =
            [&](vector<urdf::JointSharedPtr>& joints) {
                // 检查是否为空
                if (joints.size() == 1) {
                    auto joint = joints[0];
                    if (joint->type != urdf::Joint::FIXED) {
                        Joint* joint_s = new Joint();
                        joint_s->name = joint->name;
                        if (last_joint) last_joint->child_links = links;
                        links.clear();
                        joint_map_[id] = joint_s;
                        id++;
                        last_joint = joint_s;
                    }
                    auto child_link = model.getLink(joint->child_link_name);
                    link.name = child_link->name;
                    links.emplace_back(link);
                    auto child_joints = child_link->child_joints;
                    func(child_joints);
                } else if (joints.empty() || joints.size() > 1) {
                    if (last_joint) last_joint->child_links = links;
                }
            };
        func(child_joints);

        for (auto fixed_link : fixed_links_) {
            std::cout << "Link name: " << fixed_link.name << std::endl;
            auto origin_to_fixed = fixed_link.link_to_fixed.getOrigin();
            auto rotation_to_fixed = fixed_link.link_to_fixed.getRotation();
            std::cout << "position is: [" << origin_to_fixed.getX() << ", "
                      << origin_to_fixed.getY() << ", " << origin_to_fixed.getZ() << "]"
                      << std::endl;
            std::cout << "rotation is: [" << rotation_to_fixed.getX() << ", "
                      << rotation_to_fixed.getY() << ", " << rotation_to_fixed.getZ() << ", "
                      << rotation_to_fixed.getW() << "]" << std::endl;
            std::cout << fixed_link.mesh_file << std::endl;
        }
    }

    for (const auto& joint_pair : joint_map_) {
        const auto& joint = joint_pair.second;
        std::cout << "Joint name " << joint->name << std::endl;
        for (auto& child_link : joint->child_links)
            std::cout << "Link name " << child_link.name << std::endl;
        std::cout << std::endl;
    }
    is_initialized_ = true;
}

visualization_msgs::msg::Marker TrajectoryVisualization::getMarker(
    int id, const std::string ns, double alpha, const Eigen::Matrix4d& T,
    const std::string& mesh_file) {
    Eigen::Matrix3d rotation_matrix = T.block(0, 0, 3, 3);
    Eigen::Quaterniond quad;
    quad = rotation_matrix;
    visualization_msgs::msg::Marker meshMarker;
    meshMarker.header.frame_id = "map";
    meshMarker.header.stamp = clock_.now();
    meshMarker.ns = ns;
    meshMarker.id = id;
    meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    meshMarker.action = visualization_msgs::msg::Marker::ADD;
    meshMarker.mesh_use_embedded_materials = true;
    meshMarker.pose.position.x = T(0, 3);
    meshMarker.pose.position.y = T(1, 3);
    meshMarker.pose.position.z = T(2, 3);
    meshMarker.pose.orientation.w = quad.w();
    meshMarker.pose.orientation.x = quad.x();
    meshMarker.pose.orientation.y = quad.y();
    meshMarker.pose.orientation.z = quad.z();
    meshMarker.scale.x = 1.0;
    meshMarker.scale.y = 1.0;
    meshMarker.scale.z = 1.0;
    meshMarker.lifetime = rclcpp::Duration(60, 0);  
    if (alpha >= 0.0) meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    return meshMarker;
}

bool TrajectoryVisualization::hasMeshFile(
    std::shared_ptr<const urdf::Link>& link, std::string& mesh_file) {
    std::vector<urdf::Pose> origins;                       // 位姿
    std::vector<urdf::MaterialConstSharedPtr> materials;   // 颜色材质
    std::vector<urdf::GeometryConstSharedPtr> geometries;  // 几何信息

    if (geometry_type_.empty() || geometry_type_ == "visual") {
        for (const auto& visual : link->visual_array) {
            origins.emplace_back(visual->origin);
            materials.push_back(visual->material);
            geometries.push_back(visual->geometry);
        }
    } else if (geometry_type_ == "collision") {
        for (const auto& collision : link->collision_array) {
            origins.emplace_back(collision->origin);
            materials.emplace_back(nullptr);
            geometries.emplace_back(collision->geometry);
        }
    } else
        std::cerr << "Invaild geometry type " << geometry_type_ << std::endl;

    if (geometries.size() == 1) {
        const auto& origin = origins[0];
        const auto& geometry = geometries[0];
        const auto& material = materials[0];

        if (geometry->type == urdf::Geometry::MESH) {
            const urdf::MeshConstSharedPtr mesh =
                std::dynamic_pointer_cast<const urdf::Mesh>(geometry);
            if (mesh->filename.find("package://") != std::string::npos) {
                mesh_file = mesh->filename;
                return true;
            } else
                std::cerr << "Mesh file path is not correct: " << mesh->filename << std::endl;
        } else
            std::cerr << "Invaild geometry type: " << std::endl;
    }

    return false;
}

visualization_msgs::msg::MarkerArray TrajectoryVisualization::getMarkerArray(
    std::string ns, int idx, double alpha, Eigen::VectorXd state) {
    visualization_msgs::msg::MarkerArray marker_array;
    for(int i = 0; i < fixed_links_.size(); i++) {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        marker_array.markers.emplace_back(getMarker(i, ns, alpha, T, fixed_links_[i].mesh_file));
    }
    return marker_array;
}
