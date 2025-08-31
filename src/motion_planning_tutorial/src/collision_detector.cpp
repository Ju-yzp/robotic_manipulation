#include <motion_planning_tutorial/collision_detector.hpp>

#include <pcl/impl/point_types.hpp>

namespace motion_planning_tutorial {
bool CollisionDetector::isOccurCollision(const Eigen::Vector3d& center, const double radius) {
    // 暂时是通过搜寻以center为中心，raius为半径形成的球范围内的所有点云，然后如果有点云存在，就返回true
    // 后期会修改ikd-Tree,引进点云稀疏化与线性存储，降低内存使用，提高缓存命中率

    pcl::PointXYZ ball_center_pt;
    ball_center_pt.x = center(0);
    ball_center_pt.y = center(1);
    ball_center_pt.z = center(2);

    KD_TREE<pcl::PointXYZ>::PointVector search_points;
    kd_tree_->Radius_Search(ball_center_pt, radius, search_points);
    return search_points.empty() ? false : true;
}
}  // namespace motion_planning_tutorial
