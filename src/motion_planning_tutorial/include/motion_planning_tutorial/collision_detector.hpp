/*
Description: CollisionDetector: a class for detecting collisions using k-d tree
Author: Jup
email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_COLLISION_DETECTOR_HPP_
#define MOTION_PLANNING_TOTURIAL_COLLISION_DETECTOR_HPP_

// eigen
#include <Eigen/Eigen>

// motion_planning_tutorial
#include <motion_planning_tutorial/ikd_Tree.h>

// cpp
#include <memory>

template class KD_TREE<pcl::PointXYZ>;

namespace motion_planning_tutorial {

// TODO:用于测试
struct Scene {
    std::vector<Eigen::Vector4d> obstacle_centers;
    std::vector<double> obstacle_radius;
};

class CollisionDetector {
public:
    using UniquePtr = std::unique_ptr<CollisionDetector>;

    CollisionDetector(std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree) : kd_tree_(kd_tree) {}

    // 检测是否发生碰撞，是则返回true
    bool isOccurCollision(const Eigen::Vector4d& center, const double radius);

    void set_scene(const Scene& scene) { scene_ = scene; }

private:
    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;  // k-d树用于碰撞检测
    Scene scene_;  // TODO：场景信息，包含障碍物中心和半径（测试版本）
};
}  // namespace motion_planning_tutorial

#endif
