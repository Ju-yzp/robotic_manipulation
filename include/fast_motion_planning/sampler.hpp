#ifndef FAST_MOTION_PLANNING_SAMPLER_HPP_
#define FAST_MOTION_PLANNING_SAMPLER_HPP_

// fast_motion_planning
#include <fast_motion_planning/RobotParams.hpp>
#include <fast_motion_planning/types.hpp>

// cpp
#include <cstddef>
#include <iostream>
#include <random>

// eigen
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/Eigen>

namespace fast_motion_planning {

class RandomNumberGenerator {  // 随机数字生成器
public:
    double uniform01() { return uniDist_(generator_); }

    double uniform(const double upper, const double lower) {
        return lower + (upper - lower) * uniform01();
    }

    int uniformInt(const int upper, const int lower) {
        auto r = (int)floor(uniform((double)lower, (double)(upper) + 1.0));
        return (r > upper) ? upper : r;
    }

    double gaussian01() { return normalDist_(generator_); }

    double gaussian(double mean, double stddev);

    Eigen::Quaterniond quaternion();

    Eigen::Vector3d eulerRPY();

private:
    std::mt19937 generator_;

    // 正态分布
    std::normal_distribution<> normalDist_{0, 1};

    // 均匀分布
    std::uniform_real_distribution<> uniDist_{0, 1};
};

class Sampler {
public:
    REGISTER_SMART_POINTER(Sampler)

    Sampler(const RobotParams::SharedPtr& robot_params);

    Sampler() = delete;

    State sample(size_t start, size_t expand);

    State sample();

private:
    // 使用两个维度进行采样时，会记录上个维度的值
    double last_state_;

    // 随机数字生成器
    RandomNumberGenerator rng_;

    // 约束矩阵(关节位置限制)
    Eigen::MatrixXd constraints_;

    // 维数
    size_t dim_;

    // 使用空间管理者标志
    bool is_use_sn_{false};
};
}  // namespace fast_motion_planning

#endif
