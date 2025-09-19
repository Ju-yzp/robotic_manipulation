// visualization_module
#include <visualization_tools/trajectory_visualization.hpp>

// urdf
#include <urdf/model.h>
#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/pose.h>
#include <urdf_model/types.h>

// eigen
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

// cpp
#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <stack>

namespace visualization_tools {

void TrajectoryVisualization::loadModel(const string& model_description, bool show) {
    if (is_initialized_) return;

    urdf::Model model;
    if (model.initFile(model_description)) {
        auto root_link = model.getRoot();
        if (!root_link) return;

        /*
         * TODO：递归可能会造成栈溢出行为的发生，后面会把全部递归操作重写，换成循环结合模拟栈行为，
         * 使用局部变量存储信息，相对于递归会降低栈的使用容量
         */

        auto child_joints = root_link->child_joints;

        // // 递归版本(后面测试完循环版本后，会保留代码，分享一下自己的思路)
        // for (auto child_joint : child_joints) processJoint(child_joint, model, nullptr, true);

        // 循环版本
        struct JointData {
            Joint* joint;
            shared_ptr<urdf::Joint> urdf_joint;
        };
        std::stack<JointData> joint_stack;
        JointData jd;
        for (auto child_joint : child_joints) {
            Joint* joint = new Joint();
            joint->name = child_joint->name;
            root_joints_.emplace_back(joint);
            joints_.emplace_back(joint);
            jd.joint = joint;
            jd.urdf_joint = child_joint;
            joint_stack.push(jd);
            processJoint(child_joint, model, joint);
        }

        while (!joint_stack.empty()) {
            jd = joint_stack.top();
            joint_stack.pop();
            auto child_link = model.getLink(jd.urdf_joint->child_link_name);
            child_joints = child_link->child_joints;
            for (auto child_joint : child_joints) {
                JointData jd_;
                Joint* joint = new Joint();
                jd.joint->child_joints.emplace_back(joint);
                joint->name = child_joint->name;
                joints_.emplace_back(joint);
                jd_.joint = joint;
                jd_.urdf_joint = child_joint;
                joint_stack.push(jd_);
                processJoint(child_joint, model, joint);
            }
        }

        // TODO： 调试递归遍历树输出节点信息也会改成上面相同版本
        // 递归版本（warning:尽量不要使用）
        // 从根节点开始遍历
        // if (show) {
        //     cout << "--------Parse Test---------" << endl;
        //     Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        //     function<void(Joint * joint)> iter_func = [&](Joint* joint) {
        //         if (joint) {
        //             if (!joint->child_link.mesh_file.empty()) {
        //                 cout << "Joint name: " << joint->name << endl;
        //                 cout << "Child Link name: " << joint->child_link.name << endl;
        //                 cout << "Mesh file: " << joint->child_link.mesh_file << endl;
        //                 cout << joint->origin_pose.format(CleanFmt) << std::endl;
        //             }
        //             cout << endl;
        //             for (auto child_joint : joint->child_joints) iter_func(child_joint);
        //         }
        //     };
        //     for (auto root_joint : root_joints_) {
        //         iter_func(root_joint);
        //     }
        // }

        // 普通循环替代递归版本(iterative version instead of recursive version)
        if (show) {
            stack<Joint*> iter_stack;
            for (auto root_joint : root_joints_) iter_stack.push(root_joint);

            Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

            while (!iter_stack.empty()) {
                const auto& joint = iter_stack.top();
                iter_stack.pop();
                std::cout << joint->name << std::endl;
                if (!joint->child_link.mesh_file.empty()) {
                    cout << "Joint name: " << joint->name << endl;
                    cout << "Child Link name: " << joint->child_link.name << endl;
                    cout << "Mesh file: " << joint->child_link.mesh_file << endl;
                    cout << joint->child_link.pose_to_fixed.format(CleanFmt) << std::endl;
                    cout << endl;
                }

                for (auto child_joint : joint->child_joints) iter_stack.push(child_joint);
            }
        }
    } else {
        cout << __FILE__ << endl;
        cout << __LINE__ << endl;
        cerr << "Failed to parse urdf file" << endl;
        return;
    }
}

void TrajectoryVisualization::processJoint(
    const shared_ptr<urdf::Joint>& urdf_joint, const urdf::Model& model, Joint* joint) {
    if (!joint) return;

    // 获取关节位姿信息
    urdf::Pose joint_pose = urdf_joint->parent_to_joint_origin_transform;
    const auto& rot = joint_pose.rotation;
    const auto& pos = joint_pose.position;
    Eigen::Quaterniond quad(rot.w, rot.x, rot.y, rot.z);
    Eigen::Vector3d origin(pos.x, pos.y, pos.z);
    joint->origin_pose.block(0, 0, 3, 3) = quad.toRotationMatrix();
    joint->origin_pose.block(0, 3, 3, 1) = origin;
    joint->new_pose = joint->origin_pose;

    // 获取关节子连杆信息:位姿和纹理文件
    Link child_link;
    auto urdf_child_link = model.getLink(urdf_joint->child_link_name);
    child_link.name = urdf_child_link->name;
    get_link_pose(child_link, urdf_child_link);
    get_meshfile(urdf_child_link, child_link.mesh_file);
    joint->child_link = child_link;

    // 确定关节旋转类型,暂时不支持滑动和浮动类型
    static constexpr double EPS = 1e-2;
    if (urdf_joint->type != urdf::Joint::FLOATING && urdf_joint->type != urdf::Joint::PRISMATIC &&
        urdf_joint->type != urdf::Joint::FIXED) {
        const auto axis = urdf_joint->axis;
        joint->is_fixed = false;
        if (axis.x > EPS)
            joint->jra = JointRotationAxis::X;
        else if (axis.y > EPS)
            joint->jra = JointRotationAxis::Y;
        else
            joint->jra = JointRotationAxis::Z;
    }
}

// void TrajectoryVisualization::processJoint(
//     shared_ptr<urdf::Joint>& joint, const urdf::Model& model, Joint* parent_joint, bool is_root)
//     { Joint* joint_struct = new Joint(); if (is_root) root_joints_.emplace_back(joint_struct);

//     joints_.emplace_back(joint_struct);
//     joint_struct->name = joint->name;

//     // 关节位姿
//     auto pose = joint->parent_to_joint_origin_transform;
//     Eigen::Quaterniond quad(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
//     Eigen::Vector3d origin(pose.position.x, pose.position.y, pose.position.z);
//     joint_struct->origin_pose.block(0, 0, 3, 3) = quad.toRotationMatrix();
//     joint_struct->origin_pose.block(0, 3, 3, 1) = origin;
//     joint_struct->new_pose = joint_struct->origin_pose;

//     auto child_link = model.getLink(joint->child_link_name);
//     // 获取子连杆位姿信息
//     Link link;
//     link.name = child_link->name;
//     get_link_pose(link, child_link);
//     // 尝试获取子连杆的纹理文件
//     if (!get_meshfile(child_link, link.mesh_file))
//         cerr << "No mesh file for link " << link.name << endl;
//     joint_struct->child_link = link;

//     // 如果关节不是固定的(我们默认关节只有旋转/固定，不支持移动基座)
//     if (joint->type != urdf::Joint::FIXED) {
//         joint_struct->is_fixed = false;
//         auto axis = joint->axis;
//         if (axis.x > 1e-2)
//             joint_struct->jra = JointRotationAxis::X;
//         else if (axis.y > 1e-2)
//             joint_struct->jra = JointRotationAxis::Y;
//         else
//             joint_struct->jra = JointRotationAxis::Z;
//     }

//     if (parent_joint) parent_joint->child_joints.emplace_back(joint_struct);

//     auto child_joints = child_link->child_joints;
//     for (auto child_joint : child_joints) {
//         processJoint(child_joint, model, joint_struct);
//     }
// }

bool TrajectoryVisualization::get_meshfile(shared_ptr<const urdf::Link>& link, string& mesh_file) {
    urdf::GeometryConstSharedPtr geometry;
    if (geometry_type_.empty() || geometry_type_ == "visual") {
        geometry = link->visual ? link->visual->geometry : nullptr;
    } else if (geometry_type_ == "collision") {
        geometry = link->collision ? link->collision->geometry : nullptr;
    } else {
        cerr << "Invalid geometry type " << geometry_type_ << endl;
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
    const std::unordered_map<std::string, double>& joint_state_pair, bool show) {
    visualization_msgs::msg::MarkerArray marker_array;

    // 递归版本(unsafe)
    // 先更新关节位置
    // Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    // for (auto& root_joint : root_joints_) {
    //     updateState(joint_state_pair, root_joint, tf);
    // }

    // 循环版本(safe)
    struct TransformData {
        Joint* joint;
        Eigen::Matrix4d tf{Eigen::Matrix4d::Identity()};
    };
    stack<TransformData> joint_stack;
    TransformData td;

    for (const auto& root_joint : root_joints_) {
        td.tf = root_joint->origin_pose;
        td.joint = root_joint;
        joint_stack.push(td);
    }

    while (!joint_stack.empty()) {
        td = joint_stack.top();
        joint_stack.pop();
        auto& link = td.joint->child_link;
        if (joint_state_pair.find(td.joint->name) != joint_state_pair.end() &&
            !td.joint->is_fixed) {
            double new_state = joint_state_pair.find(td.joint->name)->second;
            td.joint->update_state(new_state);
        } else
            td.joint->new_pose = td.joint->origin_pose;

        // 更新关节子连杆的位姿
        td.tf *= td.joint->new_pose;
        link.pose_to_fixed = td.tf * link.offest;

        for (auto child_joint : td.joint->child_joints) {
            TransformData td_;
            td_.joint = child_joint;
            td_.tf = td.tf * td_.tf;
            joint_stack.push(td_);
        }
    }

    // 测试代码
    // if (show) {
    //     Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //     cout << "-------Update State Test--------" << std::endl;
    //     function<void(Joint * joint)> iter_func = [&](Joint* joint) {
    //         if (joint && !joint->child_link.mesh_file.empty()) {
    //             cout << "Joint name: " << joint->name << endl;
    //             cout << "Child Link name: " << joint->child_link.name << endl;
    //             cout << "Mesh file: " << joint->child_link.mesh_file << endl;
    //             cout << joint->child_link.pose_to_fixed.format(CleanFmt) << std::endl;
    //             cout << endl;
    //         }
    //         for (auto child_joint : joint->child_joints) iter_func(child_joint);
    //     };
    //     for (auto root_joint : root_joints_) {
    //         iter_func(root_joint);
    //     }
    // }

    // 循环版本
    if (show) {
        stack<Joint*> iter_stack;
        for (const auto& root_joint : root_joints_) iter_stack.push(root_joint);

        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

        while (!iter_stack.empty()) {
            const auto& joint = iter_stack.top();
            iter_stack.pop();
            if (!joint->child_link.mesh_file.empty()) {
                cout << "Joint name: " << joint->name << endl;
                cout << "Child Link name: " << joint->child_link.name << endl;
                cout << "Mesh file: " << joint->child_link.mesh_file << endl;
                cout << joint->child_link.pose_to_fixed.format(CleanFmt) << std::endl;
                cout << endl;
            }

            for (const auto& child_joint : joint->child_joints) iter_stack.push(child_joint);
        }
    }

    vector<string> joint_list;
    joint_list.reserve(joint_state_pair.size() + 2);
    std::transform(
        joint_state_pair.begin(), joint_state_pair.end(), std::back_inserter(joint_list),
        [](const auto& pair) { return pair.first; });

    vector<Link> link_list = get_links(joint_list);

    // cout << "Link size is " << int(link_list.size()) << std::endl;
    //  把marker逐个添加至marker array中
    for (uint32_t index{0}; index < link_list.size(); ++index) {
        marker_array.markers.emplace_back(getMarker(
            idx * link_list.size() + index, ns, alpha, link_list[index].pose_to_fixed,
            link_list[index].mesh_file));
        // cout << link_list[index].mesh_file << endl;
    }
    return marker_array;
}

// void TrajectoryVisualization::updateState(
//     const std::unordered_map<std::string, double>& joint_state_pair, Joint* joint,
//     Eigen::Matrix4d tf) {
//     if (!joint) return;

//     auto& link = joint->child_link;
//     if (joint_state_pair.find(joint->name) != joint_state_pair.end() && !joint->is_fixed) {
//         double new_state = joint_state_pair.find(joint->name)->second;
//         joint->update_state(new_state);
//     } else
//         joint->new_pose = joint->origin_pose;

//     // 更新关节子连杆的位姿
//     tf *= joint->new_pose;
//     link.pose_to_fixed = tf * link.offest;

//     for (auto child_joint : joint->child_joints) updateState(joint_state_pair, child_joint, tf);
// }

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

vector<TrajectoryVisualization::Link> TrajectoryVisualization::get_links(
    const vector<string>& joint_list) {
    vector<Link> link_list;
    Joint* searched_joint = nullptr;
    for (auto joint_name : joint_list) {
        if (get_joint(&searched_joint, joint_name) && !searched_joint->child_link.mesh_file.empty())
            link_list.emplace_back(searched_joint->child_link);
    }
    return link_list;
}

bool TrajectoryVisualization::get_joint(Joint** searched_joint, const string joint_name) {
    for (const auto& joint : joints_)
        if (joint->name == joint_name) {
            *searched_joint = joint;
            return true;
        }
    return false;
}
}  // namespace visualization_tools
