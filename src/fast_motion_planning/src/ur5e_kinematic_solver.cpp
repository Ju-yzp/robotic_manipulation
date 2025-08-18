#include<fast_motion_planning//ur5e_kinematic_solver.hpp>

#include<array>
#include<vector>

Eigen::Matrix4d frameTransform(float a,float d,float alpha,float theta)
{
    Eigen::Matrix4d transform_matrix;
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

Eigen::Matrix4d Ur5eKinematicSolver::get_endeffector_pose(Ur5eParam theta_table)
{
Eigen::Matrix4d endeffector_to_base = Eigen::Matrix4d::Identity();
endeffector_to_base = endeffector_to_base * 
                      frameTransform(a_table_(0),d_table_(0),alpha_table_(0),theta_table(0))*
                      frameTransform(a_table_(1),d_table_(1),alpha_table_(1),theta_table(1))*
                      frameTransform(a_table_(2),d_table_(2),alpha_table_(2),theta_table(2))*
                      frameTransform(a_table_(3),d_table_(3),alpha_table_(3),theta_table(3))*
                      frameTransform(a_table_(4),d_table_(4),alpha_table_(4),theta_table(4))*
                      frameTransform(a_table_(5),d_table_(5),alpha_table_(5),theta_table(5));
return endeffector_to_base;
};

std::vector<Eigen::VectorXd> Ur5eKinematicSolver::solveInverseKinematic(const Eigen::Matrix4d goal_end_effector_pose)
{
std::vector<Eigen::Vector<double, UR5E_DOF>> all_solutions;
all_solutions.reserve(8);

static Eigen::Matrix4d endeffector_to_frame6;

endeffector_to_frame6 << 1.0f,0.0f,0.0f,0.0f,
                         0.0f,1.0f,0.0f,0.0f,
                         0.0f,0.0f,1.0f,100.0f,
                         0.0f,0.0f,0.0f,1.0f;

Eigen::Matrix4d frame6_to_frame0 = goal_end_effector_pose * endeffector_to_frame6.inverse();

float theta1[2];

// 求第一个角度
getFirstTheta(frame6_to_frame0(1,3),frame6_to_frame0(0,3),-std::abs(d_table_(1)),theta1);

for(size_t index = 0; index < 2;index++)
{

// 将第二、三、四个角度的和视为一个角度，因为他们对于末端执行器的姿态贡献是线性关系
if(theta1[index] < -100)
   continue;

// 设置角度一的值
theta_table_(0) = -theta1[index];


// 求坐标系六到坐标系一的变换矩阵
Eigen::Matrix4d frame6_to_frame1 = frameTransform(a_table_(0), 
                                                  d_table_(0),
                                                  alpha_table_(0),
                                                  theta_table_(0)).inverse() *
                                                  frame6_to_frame0;


frame6_to_frame1(1,3) -= std::abs(d_table_(1));

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "----Frame6 To Frame0----" << std::endl;
std::cout << frame6_to_frame1.format(CleanFmt) << std::endl;

float wrist_solution[2][3];
getWristThetas(frame6_to_frame1, wrist_solution[0]);
Eigen::Matrix4d endeffector_to_base = Eigen::Matrix4d::Identity();
endeffector_to_base = endeffector_to_base * 
                      frameTransform(a_table_(1),d_table_(1),alpha_table_(1),wrist_solution[0][0])*
                      frameTransform(a_table_(4),d_table_(4),alpha_table_(4),wrist_solution[0][1])*
                      frameTransform(a_table_(5),d_table_(5),alpha_table_(5),wrist_solution[0][2]);

std::cout << "----Rotation  Matrix----" << std::endl;
std::cout << endeffector_to_base.format(CleanFmt) << std::endl;
Eigen::Matrix4d endeffector_to_base1 = Eigen::Matrix4d::Identity();
endeffector_to_base1 = endeffector_to_base1 * 
                      frameTransform(a_table_(1),d_table_(1),alpha_table_(1),wrist_solution[1][0])*
                      frameTransform(a_table_(4),d_table_(4),alpha_table_(4),wrist_solution[1][1])*
                      frameTransform(a_table_(5),d_table_(5),alpha_table_(5),wrist_solution[1][2]);

std::cout << "----Rotation  Matrix1----" << std::endl;
std::cout << endeffector_to_base1.format(CleanFmt) << std::endl;

for(size_t inner_index = 0; inner_index < 2; inner_index++)
{
std::vector<std::array<float,3>> arm_solutions;
getArmThetas(wrist_solution[inner_index][0],arm_solutions, frame6_to_frame1);


for(auto arm_solution:arm_solutions)
{
all_solutions.push_back(Eigen::Vector<double,UR5E_DOF>{-theta1[index],
                                                            arm_solution[0],
                                                            arm_solution[1],
                                                            arm_solution[2],
                                                            wrist_solution[inner_index][1],
                                                            wrist_solution[inner_index][2]});
}
}
}
std::vector<Eigen::VectorXd> x;
for (auto solution :all_solutions) {
Eigen::VectorXd b = solution;
x.emplace_back(b);
}
return x;
};

void Ur5eKinematicSolver::getFirstTheta(float A,float B,float C,float theta1[])
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

void Ur5eKinematicSolver::getWristThetas(const Eigen::Matrix4d pose,float *wrist_solution)
{
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

void Ur5eKinematicSolver::getArmThetas(float total_theta,std::vector<std::array<float,3>>& arm_solutions,
                                          Eigen::Matrix4d &pose)
{

float new_x = pose(0,3) + sin(total_theta) * std::abs(d_table_(4));
float new_z = pose(2,3) + cos(total_theta) * std::abs(d_table_(4));

float a2 = a_table_(2);
float a3 = a_table_(3);


float len = sqrt(pow(new_x,2) + pow(new_z,2));

// if(a2 + a3 < len || a2 - a3 > len)
//    std::cout<<"Cound't reach specify pose"<<std::endl;

float beta = acos((pow(len,2)+pow(a2,2)-pow(a3,2))/(2.0f*len*a2));
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