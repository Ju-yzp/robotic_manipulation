#include <cstddef>
#include <iostream>

#include <motion_planning/util.hpp>

namespace motion_planning {

Eigen::Matrix4f frameTransform(float a,float d,float alpha,float theta)
{
    Eigen::Matrix4f transform_matrix;
    float st = sin(theta);
    float ct = cos(theta);
    float sa = sin(alpha);
    float ca = cos(alpha);
    transform_matrix << ct,   -st,  0,  a,
                       st*ca,ct*ca,-sa,-sa*d,
                       st*sa,ct*sa,ca, ca*d,
                       0,    0,    0,  1;
    return transform_matrix;    
}

bool checkEndeffectorPose(const Eigen::Matrix4f target_pose,const Eigen::Matrix4f reach_pose)
{
    float pos_error = pow(target_pose(0,3)-reach_pose(0,3),2) + 
                      pow(target_pose(1,3)-reach_pose(1,3),2) + 
                      pow(target_pose(2,3)-reach_pose(2,3),2) ;

    float orient_error{0};

    for( size_t row = 0; row < 3; row++)
    {
    for( size_t col = 0; col < 3; col++)
    {
    orient_error += pow(target_pose(row,col) - reach_pose(row,col),2);
    }
    }

    constexpr float standard_pos_error = 5;
    constexpr float standard_orient_error = 0.5;
    if(standard_orient_error > orient_error && standard_pos_error > pos_error)
       return true;

    std::cout<<"The position error is "<<pos_error<<std::endl;
    std::cout<<"The orientation error is "<<orient_error<<std::endl;
    std::cout<<std::endl;
    return false;
}
}