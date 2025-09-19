#ifndef FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_
#define FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_

#include <fast_motion_planning/types.hpp>

namespace fast_motion_planning {

class CollisionDetector {
public:
    REGISTER_SMART_POINTER(CollisionDetector);

    // 检测是否发生碰撞
    bool isFreeCollision(const State& new_state);

private:
};
}  // namespace fast_motion_planning

#endif
