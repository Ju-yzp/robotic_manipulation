// cpp
#include <cstddef>
#include <iostream>

// fast_motion_planning
#include <fast_motion_planning/sampler.hpp>

namespace fast_motion_planning {
Sampler::Sampler(const RobotParams::SharedPtr& robot_params) {
    dim_ = robot_params->get_dof();

    std::cout << dim_ << std::endl;
    constraints_ = Eigen::MatrixXd::Zero(dim_, 2);

    for (size_t i{0}; i < dim_; ++i) {
        const auto jointlimit = robot_params->get_jointlimit(i);
        constraints_(i, 0) = jointlimit->position_upper;
        constraints_(i, 1) = jointlimit->position_lower;
    }
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
