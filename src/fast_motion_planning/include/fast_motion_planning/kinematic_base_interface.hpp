#ifndef FAST_MOTION_PALNNING_KINEMATIC_BASE_INTERFACE_HPP_
#define FAST_MOTION_PALNNING_KINEMATIC_BASE_INTERFACE_HPP_

#include<array>
#include<cstddef>

#include<fast_motion_planning/sampler_base_interface.hpp>

namespace fast_motion_planning {

template <std::size_t DOF,typename Scalar>
struct Solution
{
std::array<Scalar,DOF> solutions;
};

template <std::size_t DOF,typename Scalar>
class KinematicBaseInterface
{
public:

KinematicBaseInterface(){}

virtual ~KinematicBaseInterface(){}

virtual Solution<DOF,Scalar> solveInverseTransform() = 0;

private:

};
}
#endif