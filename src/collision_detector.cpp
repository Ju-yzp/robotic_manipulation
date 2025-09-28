#include <fast_motion_planning/collision_detector.hpp>

namespace fast_motion_planning {
bool CollisionDetector::isFreeCollision(const State& new_state) {
    robot_params_->update_envelope_position(new_state, 0);

    for (int i = 0; i < dof_; ++i) {
        auto envelope = robot_params_->get_envelope_position(i);
        if (!envelope.has_value()) {
            continue;
        }

        auto [ep, er] = envelope.value();

        for (int j = 0; j < ep.cols(); ++j) {
            for (int k{0}; k < obstacles_position_.cols(); ++k) {
                auto pd = (obstacles_position_.col(k) - ep.col(j)).norm();
                if (er(j) + obstacles_radius_(k) > pd) return false;
            }
        }
    }
    return true;
};
}  // namespace fast_motion_planning
