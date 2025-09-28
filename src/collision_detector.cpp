#include <fast_motion_planning/collision_detector.hpp>

namespace fast_motion_planning {
bool CollisionDetector::isFreeCollision(const State& new_state) {
    robot_params_->update_envelope_position(new_state, 0);

    // for (int i = 0; i < dof_; ++i) {
    //     auto envelope = robot_params_->get_envelope_position(i);
    //     if (!envelope.has_value()) {
    //         continue;
    //     }

    //     auto [ep, er] = envelope.value();

    //     for (int j = 0; j < ep.cols(); ++j) {
    //         Eigen::MatrixXd pd = obstacles_position_.colwise() - ep.col(j);
    //         Eigen::VectorXd rd = obstacles_radius_.array() + er(j);

    //         for (int k = 0; k < obstacles_position_.cols(); ++k) {
    //             double distance = pd.col(k).norm();

    //             if (distance < rd(k)) {
    //                 return false;
    //             }
    //         }
    //     }
    // }

    return true;
};
}  // namespace fast_motion_planning
