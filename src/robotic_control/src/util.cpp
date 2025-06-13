#include<array>
#include<cmath>
#include<iostream>
#include<robot_control/util.h>

Eigen::Matrix4f axisTransform(float a,float d,float alpha,float theta)
{
    Eigen::Matrix4f tranform_matrix;
    float st = sin(theta);
    float ct = cos(theta);
    float sa = sin(alpha);
    float ca = cos(alpha);
    tranform_matrix << ct,   -st,  0,  a,
                       st*ca,ct*ca,-sa,-sa*d,
                       st*sa,ct*sa,ca, ca*d,
                       0,    0,    0,  1;
    return tranform_matrix;
}

Eigen::Matrix4f axisTransform(const std::array<float,4> &param)
{
    Eigen::Matrix4f tranform_matrix;
    float st = sin(param[3]);
    float ct = cos(param[3]);
    float sa = sin(param[2]);
    float ca = cos(param[2]);
    tranform_matrix << ct,   -st,  0,  param[0],
                       st*ca,ct*ca,-sa,-sa*param[1],
                       st*sa,ct*sa,ca, ca*param[1],
                       0,    0,    0,  1;
    return tranform_matrix;
}

Eigen::Matrix4f forwardKinematics(const DH_Table &dh_table)
{
    Eigen::Matrix4f end_to_base  = Eigen::Matrix4f::Identity();

    for(auto &params:dh_table)
    {
        end_to_base *= axisTransform(params[0],params[1],params[2],params[3]);
    }
    return end_to_base;
}

bool checkSolutions(Eigen::Matrix4f pose,Eigen::Matrix4f reach_pose,float pos_error_upper,float rot_error_upper)
{
    bool status{false};
    
    float pos_error = std::abs(pose(0,3)-reach_pose(0,3))+
                      std::abs(pose(1,3)-reach_pose(1,3))+
                      std::abs(pose(2,3)-reach_pose(2,3));
    
    Eigen::Matrix3f real_rotation_matrix,target_rotation_matrix;
    real_rotation_matrix<<reach_pose(0,0),reach_pose(0,1),reach_pose(0,2),
                          reach_pose(1,0),reach_pose(1,1),reach_pose(1,2),
                          reach_pose(2,0),reach_pose(2,1),reach_pose(2,2);
    
    target_rotation_matrix<<pose(0,0),pose(0,1),pose(0,2),
                            pose(1,0),pose(1,1),pose(1,2),
                            pose(2,0),pose(2,1),pose(2,2);

    Eigen::Quaternionf real_quaternion(real_rotation_matrix);
    Eigen::Quaternionf target_quaternion(target_rotation_matrix);

    real_quaternion.normalized();
    target_quaternion.normalized();
    float rot_error = std::pow(real_quaternion.x()-target_quaternion.x(),2)+
                      std::pow(real_quaternion.y()-target_quaternion.y(),2)+
                      std::pow(real_quaternion.z()-target_quaternion.z(),2)+
                      std::pow(real_quaternion.w()-target_quaternion.w(),2);
    std::cout<<"position error: "<<pos_error<<std::endl;
    if(pos_error < pos_error_upper )//&& rot_error < rot_error_upper)
        status = true;
    
    return status;
}