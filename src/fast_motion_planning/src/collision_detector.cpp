#include <cstddef>
#include<fast_motion_planning/collision_detector.hpp>


namespace fast_motion_planning {

CollisionDetector::CollisionDetector(RobotDescription<double>::SharedPtr robot_description,Scene& scene):
robot_description_(robot_description),
scene_(scene)
{}

// bool CollisionDetector::is_selfcollision()
// {

// }

bool CollisionDetector::is_collision_with_obstacles()
{
auto obstable_num = scene_.obstacles_position.size();

for(auto name:name_map_)
{
for(const auto& envelope:robot_description_->get_envelopes(name.second))
{
for(std::size_t id{0}; id < obstable_num; ++id)
{
auto is_collision = (envelope.last_position - scene_.obstacles_position[id]).norm() <
                          (envelope.radius + scene_.radius[id]);
if(is_collision) return true;
}
}
}
return false;
};
}