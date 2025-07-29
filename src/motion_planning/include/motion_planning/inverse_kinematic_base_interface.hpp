#ifndef MOTION_PLANNING_INVERSE_KINEMATIC_BASE_INTERFACE_HPP_
#define MOTION_PLANNING_INVERSE_KINEMATIC_BASE_INTERFACE_HPP_

#include <cstddef>

#include <Eigen/Eigen>

#include <motion_planning/util.hpp>

namespace motion_planning {

template <std::size_t DOF>

class InverseKinematicBaseInterface
{
public:
InverseKinematicBaseInterface(){}

virtual ~InverseKinematicBaseInterface(){}

virtual Solutions<DOF> inverseKinematic(Eigen::Matrix4f target_pose) = 0;

// 
virtual Eigen::VectorXf get_joint_velocity(Eigen::VectorXf end_effector_position_velocity) = 0;
};
}

#endif