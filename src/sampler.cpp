// cpp
#include <cstddef>
#include <iostream>

// fast_motion_planning
#include <fast_motion_planning/sampler.hpp>

namespace fast_motion_planning {
Sampler::Sampler(const RobotParams::SharedPtr& robot_params) {
    dim_ = robot_params->get_dof();

    constraints_ = Eigen::MatrixXd::Zero(dim_, 2);

    for (size_t i{0}; i < dim_; ++i) {
        const auto jointlimit = robot_params->get_jointlimit(i);
        constraints_(i, 0) = jointlimit->position_upper;
        constraints_(i, 1) = jointlimit->position_lower;
    }
}

Eigen::Quaterniond RandomNumberGenerator::quaternion() {
    double x0 = uniDist_(generator_);
    double r1 = sqrt(1.0 - x0), r2 = sqrt(x0);
    double t1 = 2.0 * M_PI * uniDist_(generator_), t2 = 2.0 * M_PI * uniDist_(generator_);
    double c1 = cos(t1), s1 = sin(t1);
    double c2 = cos(t2), s2 = sin(t2);

    return Eigen::Quaterniond(s1 * r1, c1 * r1, s2 * r2, c2 * r2);
}

Eigen::Vector3d RandomNumberGenerator::eulerRPY() {
    Eigen::Vector3d rqy;
    rqy[0] = M_PI * (-2.0 * uniDist_(generator_) + 1.0);
    rqy[1] = acos(1.0 - 2.0 * uniDist_(generator_)) - M_PI / 2.0;
    rqy[2] = M_PI * (-2.0 * uniDist_(generator_) + 1.0);

    return rqy;
}

State Sampler::sample(size_t start, size_t expand) {
    if (start + expand > dim_) throw;

    State state = Eigen::VectorXd::Zero(expand);
    for (size_t i{start}; i < start + expand; ++i)
        state(i - start) = rng_.uniform(constraints_(i, 0), constraints_(i, 1));

    return state;
};

State Sampler::sample() {
    State state = Eigen::VectorXd::Zero(dim_);

    for (size_t i{0}; i < dim_; ++i)
        state(i) = rng_.uniform(constraints_(i, 0), constraints_(i, 1));
    return state;
}

}  // namespace fast_motion_planning
