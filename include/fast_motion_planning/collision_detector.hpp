#ifndef FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_
#define FAST_MOTION_PLANNING_COLLISION_DETECTOR_HPP_

#include <cassert>
#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/types.hpp>

#include <Eigen/Eigen>

namespace fast_motion_planning {

class CollisionDetector {
public:
    REGISTER_SMART_POINTER(CollisionDetector);

    CollisionDetector(RobotParams::SharedPtr robot_params) : robot_params_(robot_params) {
        dof_ = robot_params_->get_dof();
    }

    // 检测是否发生碰撞
    bool isFreeCollision(const State& new_state);

    // 设置障碍物
    void set_obstacles(const Eigen::MatrixXd opn, const Eigen::VectorXd ors) {
        obstacles_position_ = opn;
        obstacles_radius_ = ors;
    }

    RobotParams::SharedPtr get_robot_params() { return robot_params_; }

    void set_oder(Eigen::VectorXi oder) {
        assert(oder.size() == dof_);
        oder_ = oder;
    }

private:
    RobotParams::SharedPtr robot_params_;

    // 球形障碍物位置
    Eigen::MatrixXd obstacles_position_;

    // 球形障碍物半径
    Eigen::VectorXd obstacles_radius_;

    int dof_;

    // 碰撞顺序
    Eigen::VectorXi oder_;
};
}  // namespace fast_motion_planning

#endif
