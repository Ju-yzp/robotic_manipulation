#include <array>
#include <cmath>
#include <iostream>

#include <motion_planning/inverse_kinematic_solver.hpp>

#include <Eigen/Dense>

namespace motion_planning {

InverseKinematicSolver::InverseKinematicSolver(std::shared_ptr<RobotModel<UR5E_DOF>> &robot_model)
:robot_model_(robot_model)
{}

InverseKinematicSolver::~InverseKinematicSolver()
{}

// 解析解解法
Solutions<UR5E_DOF> InverseKinematicSolver::inverseKinematic(Eigen::Matrix4f target_pose)
{
Solutions<UR5E_DOF> all_solutions;
all_solutions.solutions_.reserve(8);

Eigen::Matrix4f frame6_to_frame0 = target_pose;

float theta1[2];

// 求第一个角度
getFirstTheta(frame6_to_frame0(1,3),frame6_to_frame0(0,3),-std::abs(robot_model_->get_d(1)),theta1);

for(size_t index = 0; index < 2;index++)
{

// 将第二、三、四个角度的和视为一个角度，因为他们对于末端执行器的姿态贡献是线性关系
if(theta1[index] < -100)
   continue;

// 设置角度一的值
robot_model_->set_theta(-theta1[index],0);


// 求坐标系六到坐标系一的变换矩阵
Eigen::Matrix4f frame6_to_frame1 = frameTransform(robot_model_->get_a(0), 
                                                  robot_model_->get_d(0),
                                                  robot_model_->get_alpha(0),
                                                  robot_model_->get_theta(0)).inverse() *
                                                  frame6_to_frame0;


frame6_to_frame1(1,3) -= std::abs(robot_model_->get_d(1));

// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
// std::cout << frame6_to_frame1.format(CleanFmt) << std::endl << std::endl;
float wrist_solution[2][3];
getWristThetas(frame6_to_frame1, wrist_solution[0]);


for(size_t inner_index = 0; inner_index < 2; inner_index++)
{
std::vector<std::array<float,3>> arm_solutions;
getArmThetas(wrist_solution[inner_index][0],arm_solutions, frame6_to_frame1);
for(auto arm_solution:arm_solutions)
{
all_solutions.solutions_.push_back(std::array<float,UR5E_DOF>{-theta1[index],
                                                            arm_solution[0],
                                                            arm_solution[1],
                                                            arm_solution[2],
                                                            wrist_solution[inner_index][1],
                                                            wrist_solution[inner_index][2]});
// std::cout<<"theta1 "<<-theta1[index]<<std::endl;
// std::cout<<"theta2 "<<arm_solution[0]<<std::endl;
// std::cout<<"theta3 "<<arm_solution[1]<<std::endl;
// std::cout<<"theta4 "<<arm_solution[2]<<std::endl;
// std::cout<<"theta5 "<<wrist_solution[inner_index][1]<<std::endl;
// std::cout<<"theta6 "<<wrist_solution[inner_index][2]<<std::endl;
// std::cout<<std::endl;
}
}
}

return all_solutions;
};

void InverseKinematicSolver::getFirstTheta(float A,float B,float C,float theta1[])
{
    float delta = pow(A,2)+pow(B,2)-pow(C,2);

    // 公用部分
    float p = sqrt(delta);
    float v = pow(A,2)+pow(B,2);
    float y,x;
    // 处理 B = 0的情况

    theta1[0] = -100;
    theta1[1] = -100;

    if(B == 0)
    {
    if (delta > 0 ) {
        x = -C/A;
        y= -sqrt(pow(A,2)-pow(C,2))/A;
        theta1[0] = atan2(y,x);
        x = -C/A;
        y = sqrt(pow(A,2)-pow(C,2))/A;
        theta1[1] = atan2(y,x);
    }
    else if(delta == 0)
    {
        x = (-A*C)/v;
        y = (-B*C)/v;
        theta1[0] = atan2(y,x);

    }
    }
    else{ // B 不等于 0
    if(delta > 0)
    {
        x = (-A*C+B*p)/v;
        y = (-B*C-A*p)/v;
        theta1[0] = atan2(y,x);
        x = (-A*C-B*p)/v;
        y= (-B*C+A*p)/v;
        theta1[1] = atan2(y,x);

    }
    else if(delta == 0)
    {
        x = (-A*C)/v;
        y = (-B*C)/v;
        theta1[0] = atan2(y,x);

    }
    }
}

void InverseKinematicSolver::getWristThetas(const Eigen::Matrix4f pose,float *wrist_solution)
{
/**
 * 旋转矩阵结构:
 * ┌───────────────────────┬───────────────────┬───────────────────────┐
 * │ c4*c5*c6 - s2*s6      │ -s5*c6            │ c2*s6 + s4*c5*c6      │
 * ├───────────────────────┼───────────────────┼───────────────────────┤
 * │ -c4*c5*s6 - s4*c6     │ s5*s6             │ c4*c6 - s4*c5*s6      │
 * ├───────────────────────┼───────────────────┼───────────────────────┤
 * │ c4*s5                 │ c5                │ s4*s5                 │
 * └───────────────────────┴───────────────────┴───────────────────────┘
 */
if(pow(pose(1,2),2) < 1E-3)
{
wrist_solution[0] = atan2(pose(2,2),-pose(0,2));
wrist_solution[1] = M_PIf/2.0f;
wrist_solution[2] = atan2(-pose(1,1),pose(1,0));

wrist_solution[3] = atan2(-pose(2,2),pose(0,2));
wrist_solution[4] = -M_PIf/2.0f;
wrist_solution[5] = atan2(pose(1,1),-pose(1,0));
}

else if(std::abs(pow(pose(1,2),2)-1.0f) < 1E-3)
{
wrist_solution[0] = atan2(-pose(0,1),pose(0,0));
wrist_solution[1] = 0.0f;
wrist_solution[2] = 0.0f;

wrist_solution[3] = 0.0f;
wrist_solution[4] = 0.0f;
wrist_solution[5] = atan2(-pose(0,1),pose(0,0));
}
else
{
wrist_solution[1] = atan2(sqrt(pow(pose(0,2),2)+pow(pose(2,2),2)),pose(1,2));
wrist_solution[0] = atan2(pose(2,2)/sin(wrist_solution[1]),-pose(0,2)/sin(wrist_solution[1]));
wrist_solution[2] = atan2(-pose(1,1)/sin(wrist_solution[1]),pose(1,0)/sin(wrist_solution[1]));

wrist_solution[4] = atan2(-sqrt(pow(pose(0,2),2)+pow(pose(2,2),2)),pose(1,2));
wrist_solution[3] = atan2(pose(2,2)/sin(wrist_solution[4]),-pose(0,2)/sin(wrist_solution[4]));
wrist_solution[5] = atan2(-pose(1,1)/sin(wrist_solution[4]),pose(1,0)/sin(wrist_solution[4]));
}

}

void InverseKinematicSolver::getArmThetas(float total_theta,std::vector<std::array<float,3>>& arm_solutions,
                                          Eigen::Matrix4f &pose)
{

float new_x = pose(0,3) + sin(total_theta) * std::abs(robot_model_->get_d(4));
float new_z = pose(2,3) + cos(total_theta) * std::abs(robot_model_->get_d(4));

// 坐标系变换方程：
// [392 450] * | cos(θ23)  sin(θ23) | = [new_x new_z]
//             | cos(θ2)   sin(θ2)  |

float a2 = robot_model_->get_a(2);
float a3 = robot_model_->get_a(3);


float len = sqrt(pow(new_x,2) + pow(new_z,2));

// if(a2 + a3 < len || a2 - a3 > len)
//    std::cout<<"Cound't reach specify pose"<<std::endl;

float beta = acos((pow(len,2)+pow(a2,2)-pow(a3,2))/(2.0f*len*a2));
// float alpha = M_PIf - acos((pow(a2,2)+pow(a3,2)-pow(len,2))/(2.0f*a2*a3));
float gama = atan2(new_z,new_x);

if(std::isnan(gama))
   gama = 1.57f;

if(std::isnan(beta))
   beta = 1.57f;


float x1 = (new_x - a2 * cos(beta-gama))/a3;
float y1 =  (new_z + a2 * sin(beta-gama))/-a3;
float theta3_1 = atan2(y1,x1) - beta + gama;

// Case1:
arm_solutions.push_back(std::array<float,3>{beta-gama,theta3_1,total_theta-beta+gama-theta3_1});

float x2 = (new_x - a2 * cos(-beta-gama))/a3;
float y2 =  (new_z + a2 * sin(-beta-gama))/-a3;
float theta3_2 = atan2(y2,x2) + beta + gama;

// Case2:
arm_solutions.push_back(std::array<float,3>{-beta-gama,theta3_2,total_theta+beta+gama-theta3_2});

}
}