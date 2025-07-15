#ifndef MOTION_PLANNING_SCENCE_HPP_
#define MOTION_PLANNING_SCENCE_HPP_

#include <motion_planning/inverse_kinematic_base_interface.hpp>
#include <motion_planning/util.hpp>

#include <memory>
#include <vector>

namespace motion_planning {
struct Obstacble
{
virtual bool isCollision() = 0;
};

struct SphereObstacle:public Obstacble
{
double x,y,z;
double radius;
bool isCollision()override
{
return true;
}
};

// 场景
class Scence
{
public:

// 添加单个障碍物
void add_obstacle(std::shared_ptr<Obstacble> obstacble);

// 添加障碍物集
void add_obstacles(std::vector<std::shared_ptr<Obstacble>> obstacbles);

// 得到所有障碍物
const std::vector<std::shared_ptr<Obstacble>>& get_obstacles();

private:

// 障碍物集
std::vector<std::shared_ptr<Obstacble>> obstacles_;
};
}

#endif