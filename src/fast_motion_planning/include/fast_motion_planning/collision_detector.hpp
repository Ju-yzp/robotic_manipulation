/* COLLISION DETECTOR.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月16日 20:02:15
 * Last edited:
 *   YYYY-08-2025年8月20日 23:01:00
 * Auto updated?
 *   Yes
 *
 * Description:
 *   用于检测机械臂自碰撞以及与障碍物碰撞检测,目前简单使用球型障碍物进行碰撞检测，后面会使用改进版KD树
**/

#ifndef FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_
#define FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_

// fast motion plannig
#include<fast_motion_planning//robot_description.hpp>

// cpp
#include<cstddef>
#include<unordered_map>

namespace fast_motion_planning {

struct Scene
{
std::vector<Eigen::Vector4d> obstacles_position;
std::vector<double> radius;
};

class CollisionDetector
{
public:

CollisionDetector(RobotDescription<double>::SharedPtr robot_description,Scene& scene);

//bool is_selfcollision();
void set_name(const std::size_t id, std::string name){ name_map_[id] = name; }

bool is_collision_with_obstacles();

private:   

// 机器人描述,通过共享一块内存
RobotDescription<double>::SharedPtr robot_description_;

// 名称映射表
std::unordered_map<std::size_t, std::string> name_map_;

// 静态环境(测试)
Scene scene_;
};
}
#endif