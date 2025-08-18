/* COLLISION DETECTOR.hpp
 *   by Jup
 *
 * Created:
 *   YYYY-08-2025年8月16日 20:02:15
 * Last edited:
 *   YYYY-08-2025年8月16日 20:10:28
 * Auto updated?
 *   Yes
 *
 * Description:
 *   用于检测机械臂自碰撞以及与障碍物碰撞检测
**/

#ifndef FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_
#define FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_

namespace fast_motion_planning {

class CollisionDetector
{
public:

bool is_selfcollision();

bool is_collision_with_obstacles();

private:   
};
}
#endif