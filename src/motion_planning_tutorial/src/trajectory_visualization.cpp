// visualization_module
#include <visualization_module/trajectory_visualization.hpp>

// urdf
#include <Eigen/src/Geometry/Quaternion.h>
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

// cpp
#include <functional>
#include <iostream>
#include <memory>

namespace visualization_utils {

void TrajectoryVisualization::loadModel(const string& model_description) {
    if (is_initialized_) return;

    urdf::Model model;
    if (model.initFile(model_description)) {
        auto root_link = model.getRoot();
        if (!root_link) return;

        auto child_joints = root_link->child_joints;
        for (auto child_joint : child_joints) processJoint(child_joint, model, nullptr, true);

        // 从根节点开始遍历
        // cout<<"--------Parse Test---------"<<endl;
        // function<void(Joint * joint)> iter_func = [&](Joint* joint) {
        //     if (joint) {
        //         cout << "Joint name: " << joint->name << endl;
        //         cout << "Child Link name: " << joint->child_link.name << endl;
        //         cout << "Mesh file: " << joint->child_link.mesh_file << endl;
        //         cout << endl;
        //         for (auto child_joint : joint->child_joints) iter_func(child_joint);
        //     }
        // };
        // for (auto root_joint : root_joints_) {
        //     iter_func(root_joint);
        // }
    } else {
        cerr << "Failed to parse urdf file" << endl;
        return;
    }
}

void TrajectoryVisualization::processJoint(
    shared_ptr<urdf::Joint>& joint, const urdf::Model& model, Joint* parent_joint, bool is_root) {
    Joint* joint_struct = new Joint();
    if (is_root) root_joints_.emplace_back(joint_struct);

    joints_.emplace_back(joint_struct);
    joint_struct->name = joint->name;

    // 关节位姿
    auto pose = joint->parent_to_joint_origin_transform;
    Eigen::Quaterniond quad(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
    Eigen::Vector3d origin(pose.position.x, pose.position.y, pose.position.z);
    joint_struct->origin_pose.block(0, 0, 3, 3) = quad.toRotationMatrix();
    joint_struct->origin_pose.block(0, 3, 3, 1) = origin;

    auto child_link = model.getLink(joint->child_link_name);
    // 获取子连杆位姿信息
    Link link;
    link.name = child_link->name;

    // 尝试获取子连杆的纹理文件
    if (!get_meshfile(child_link, link.mesh_file))
        cerr << "No mesh file for link " << link.name << endl;
    joint_struct->child_link = link;

    // 如果关节不是固定的(我们默认关节只有旋转/固定，不支持移动基座)
    if (joint->type != urdf::Joint::FIXED) {
        joint_struct->is_fixed = false;
        auto axis = joint->axis;
        if (axis.x)
            joint_struct->jra = JointRotationAxis::X;
        else if (axis.y)
            joint_struct->jra = JointRotationAxis::Y;
        else
            joint_struct->jra = JointRotationAxis::Z;
    }

    if (parent_joint) parent_joint->child_joints.emplace_back(joint_struct);

    auto child_joints = child_link->child_joints;
    for (auto child_joint : child_joints) {
        processJoint(child_joint, model, joint_struct);
    }
}

bool TrajectoryVisualization::get_meshfile(shared_ptr<const urdf::Link>& link, string& mesh_file) {
    urdf::GeometryConstSharedPtr geometry;
    if (geometry_type_.empty() || geometry_type_ == "visual") {
        geometry = link->visual ? link->visual->geometry : nullptr;
    } else if (geometry_type_ == "collision") {
        geometry = link->collision ? link->collision->geometry : nullptr;
    } else {
        cerr << "Invaild geometry type " << geometry_type_ << endl;
        return false;
    }

    if (!geometry) return false;

    if (geometry->type == urdf::Geometry::MESH) {
        const urdf::MeshConstSharedPtr mesh = std::dynamic_pointer_cast<const urdf::Mesh>(geometry);
        mesh_file = mesh->filename;
    } else {
        cerr << "Unsupported geometry type" << geometry_type_ << endl;
        return false;
    }
    return true;
}

visualization_msgs::msg::Marker TrajectoryVisualization::getMarker(
    int id, const string ns, double alpha, const Eigen::Matrix4d& T, const string& mesh_file) {
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
    // meshMarker.lifetime = rclcpp::Duration(60, 0);
    if (alpha >= 0.0) meshMarker.color.a = alpha;
    meshMarker.mesh_resource = mesh_file;
    return meshMarker;
}

visualization_msgs::msg::MarkerArray TrajectoryVisualization::getMarkerArray(
    string ns, int idx, double alpha,
    const std::unordered_map<std::string, double>& joint_state_pair) {
    visualization_msgs::msg::MarkerArray marker_array;

    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    for (auto& root_joint : root_joints_) {
        updateState(joint_state_pair, root_joint, tf);
    }
    // 测试代码
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    cout << "-------Update State Test--------" << std::endl;
    function<void(Joint * joint)> iter_func = [&](Joint* joint) {
        if (joint) {
            // if(!joint->child_link.mesh_file.empty()){
            cout << "Joint name: " << joint->name << endl;
            cout << "Child Link name: " << joint->child_link.name << endl;
            cout << "Mesh file: " << joint->child_link.mesh_file << endl;
            cout << joint->child_link.pose_to_fixed.format(CleanFmt) << std::endl;
            cout << endl;
            for (auto child_joint : joint->child_joints) iter_func(child_joint);
            //}
        }
    };
    for (auto root_joint : root_joints_) {
        iter_func(root_joint);
    }
    return marker_array;
}

void TrajectoryVisualization::updateState(
    const std::unordered_map<std::string, double>& joint_state_pair, Joint* joint,
    Eigen::Matrix4d& tf) {
    if (!joint) return;

    auto& link = joint->child_link;
    if (joint_state_pair.find(joint->name) != joint_state_pair.end() && !joint->is_fixed) {
        double new_state = joint_state_pair.find(joint->name)->second;
        joint->update_state(new_state);
    }

    // 更新关节子连杆的位姿
    tf *= joint->origin_pose;
    link.pose_to_fixed = tf * link.offest;

    for (auto child_joint : joint->child_joints) updateState(joint_state_pair, child_joint, tf);
}

void TrajectoryVisualization::get_link_pose(Link& link, shared_ptr<const urdf::Link>& urdf_link) {
    urdf::Pose origin;
    if (geometry_type_.empty() || geometry_type_ == "visual") {
        if (urdf_link->visual)
            origin = urdf_link->visual->origin;
        else
            return;
    } else if (geometry_type_ == "collision") {
        if (urdf_link->collision)
            origin = urdf_link->collision->origin;
        else
            return;
    } else {
        std::cerr << "Unsupported geometry type" << geometry_type_ << std::endl;
        return;
    }

    link.offest(0, 3) = origin.position.x;
    link.offest(1, 3) = origin.position.y;
    link.offest(2, 3) = origin.position.z;

    Eigen::Quaterniond quad(
        origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z);
    link.offest.block(0, 0, 3, 3) = quad.toRotationMatrix();
}
}  // namespace visualization_utils
