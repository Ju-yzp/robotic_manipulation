#ifndef FAST_MOTION_PLANNING_SAMPLER_HPP_
#define FAST_MOTION_PLANNING_SAMPLER_HPP_

#include <fast_motion_planning/types.hpp>

namespace fast_motion_planning {

class Sampler {
public:
    REGISTER_SMART_POINTER(Sampler)

    State sample(uint32_t index);

private:
    // 使用两个维度进行采样时，会记录上个维度的值
    double last_state_;
};
}  // namespace fast_motion_planning

#endif
