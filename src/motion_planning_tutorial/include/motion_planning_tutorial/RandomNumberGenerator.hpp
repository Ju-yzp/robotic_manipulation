/*
Description: RangeNumberGenerator: a class for generating random numbers within a specified range
Author: Jup email: Jup230551@outlook.com
*/

#ifndef MOTION_PLANNING_TOTURIAL_RANGE_NUMBER_GENERATOR_HPP_
#define MOTION_PLANNING_TOTURIAL_RANGE_NUMBER_GENERATOR_HPP_

// cpp
#include <random>

namespace motion_planning_tutorial {
class RangeNumberGenerator {
public:

double uniform01() { return uniDist_(generator_); }

double uniform(const double lower, const double upper) {
    return lower + (upper - lower) * uniform01();
}

private:

std::mt19937 generator_;

std::uniform_real_distribution<> uniDist_{0,1};
};
}
#endif  // MOTION_PLANNING_TOTURIAL_RANGE_NUMBER_GENERATOR_HPP_