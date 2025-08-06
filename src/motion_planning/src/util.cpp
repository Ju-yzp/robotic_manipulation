#include <cstddef>
#include <iostream>

#include <motion_planning/util.hpp>
#include <stdexcept>

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

void SVDSolver::solve(Eigen::MatrixXd A,Eigen::VectorXd B)
{
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
// 先求得一个
// if(A.cols() > A.rows())
//   throw std::runtime_error("Rows must larger than cols that input matrix");

U_ = Eigen::MatrixXd::Identity(A.rows(),A.rows());
V_ = Eigen::MatrixXd::Identity(A.cols(),A.cols());

// 左侧变换
for(size_t i{0}; i < A.rows() - 1; i++)
{
Eigen::VectorXd v = A.col(i).tail(A.rows() - i );

// 取符号
v(0,0) = (v(0,0) >= 0 ? v(0,0) + v.norm() : v(0,0) - v.norm());

Eigen::MatrixXd H = ( 2 * v * v.transpose()) / (v.transpose() * v) * -1.0f;
for(size_t j{0}; j < H.rows(); j++)
   H(j,j) = 1.0f + H(j,j);

if( i != 0){
Eigen::MatrixXd h(A.rows(),A.rows());
h<<Eigen::MatrixXd::Identity(i,i),Eigen::MatrixXd::Zero(i,A.rows() - i),
   Eigen::MatrixXd::Zero(A.rows() - i,i),H;
A = h * A;
U_ = U_ * h;
}
else {
A = H * A;
U_ = U_ * H;
}
}

for(size_t i{0}; i < A.rows(); i++)
{
for(size_t j{0}; j < A.cols(); j++)
{
if(abs(A(i,j)) < error_)
  A(i,j) = 0.0f;
}
}
// std::cout << A.format(CleanFmt) << std::endl << std::endl;

// 右侧变换
for(size_t i{0}; i < A.cols() - 1; i++)
{
Eigen::RowVectorXd v = A.row(i).segment(i+1,A.cols());
std::cout << v.format(CleanFmt) << std::endl << std::endl;
}
}
}