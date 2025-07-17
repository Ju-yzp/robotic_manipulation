#include <motion_planning/scence.hpp>

namespace motion_planning {

void Scence::add_obstacle(std::shared_ptr<Obstacble> obstacble)
{
obstacles_.emplace_back(obstacble);
}

void Scence::add_obstacles(std::vector<std::shared_ptr<Obstacble>> obstacbles)
{
obstacles_.insert(obstacbles.begin(),obstacbles.end(),obstacles_.end());
}

const std::vector<std::shared_ptr<Obstacble>>& Scence::get_obstacles()const
{
return obstacles_;
}
}