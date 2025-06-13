#ifndef ROBOTIC_CONTROL_INVERSE_KINEMATICS_INTERFACES_H__
#define ROBOTIC_CONTROL_INVERSE_KINEMATICS_INTERFACES_H__

#include<cstdint>
#include<vector>

#include<eigen3/Eigen/Eigen>
#include<eigen3/Eigen/Dense>

typedef  Eigen::Matrix4f  Pose;

namespace robot_interfaces {

template<uint8_t Num>
struct Solutions
{
    float theta[Num];
};

template <uint8_t DOF>
class InverseKinematicsInterface
{
public:
InverseKinematicsInterface()=default;

~InverseKinematicsInterface()=default;

virtual std::vector<Solutions<DOF>> inverseKinematics(const Pose end_pose) = 0;
};
}

#endif