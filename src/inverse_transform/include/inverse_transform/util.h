#ifndef ROBOTIC_CONTROL_UTIL_H__
#define ROBOTIC_CONTROL_UTIL_H__

#include<array>
#include<eigen3/Eigen/Eigen>
#include<eigen3/Eigen/Dense>

#include<vector>

typedef std::vector<std::array<float,4>> DH_Table;

Eigen::Matrix4f axisTransform(float a,float b,float alpha,float theta);

Eigen::Matrix4f axisTransform(const std::array<float,4> &param);

Eigen::Matrix4f forwardKinematics(const DH_Table &dh_table);

bool checkSolutions(Eigen::Matrix4f pose,Eigen::Matrix4f reach_pose,float pos_error_upper,float rot_error_upper);

#endif