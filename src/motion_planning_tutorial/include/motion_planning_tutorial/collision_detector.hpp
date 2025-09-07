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
class CollisionDetector {
public:
    using UniquePtr = std::unique_ptr<CollisionDetector>;

    CollisionDetector(std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree) : kd_tree_(kd_tree) {}

    // 检测是否发生碰撞，是则返回true
    bool isOccurCollision(const Eigen::Vector4d& center, const double radius);

private:
    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;  // k-d树用于碰撞检测
};
}  // namespace motion_planning_tutorial

#endif
