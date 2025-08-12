// fast_motion_planning
#include<fast_motion_planning/kinematic_base_interface.hpp>
#include<fast_motion_planning/joint_limit_group.hpp>
#include<fast_motion_planning/envelope_group.hpp>

// cpp
#include<functional>
#include<array>

namespace fmp = fast_motion_planning;
class TwoLinkRobot:public fmp::KinematicBaseInterface<2,double>
{
public:
TwoLinkRobot(const std::string joint_config_file,const std::string envelope_config_file)
:joint_limit_group_(joint_config_file),
envelope_group_(envelope_config_file)
{
a_table_ = {100.0f,80.0f};
d_table_ = {0.0f,0.0f,};
alpha_table_ = {0.0f,0.0f};
theta_table_ = {0.0f,0.0f};
}

TwoLinkRobot(fmp::JointLimitGroupd joint_limit_group,fmp::EnvelopeGroupd envelope_group)
:joint_limit_group_(joint_limit_group),
envelope_group_(envelope_group)
{}

std::array<double, 2> solveInverseTransform(const Pose target_pose)override
{

std::array<double, 2> best_solution;

const double& a1 = a_table_[0];
const double& a2 = a_table_[1];
const double dist = sqrt(pow(target_pose(0,3),2)+pow(target_pose(1,3),2));
if( dist > (a1+a2))
  {
    std::cout<<"robot arm don't reach target position"<<std::endl;
    return best_solution;
  }
static Eigen::Vector2d weight;
weight << 0.7f,0.3f;

// 通用的条件
double alpha,beta,gama;
alpha = atan2(target_pose(1,3),target_pose(0,3));
gama = acos((pow(a1,2) + pow(dist,2) - pow(a2,2)) / (2.0f * a1 * dist));
beta = acos((pow(a1,2) + pow(a2,2) - pow(dist,2)) / (2.0f * a1 * a2));

// 第一种解
double theta1_1,theta2_1;
theta1_1 = alpha - gama;
theta2_1 = M_PIf - beta;

double theta1_2,theta2_2;
theta1_2 = alpha + gama;
theta2_2 = beta - M_PIf;


Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

Eigen::Vector2d last_joint_status,status_1,status_2;
last_joint_status<<1.1f,0.2f;
status_1<<theta1_1,theta2_1;
status_2<<theta1_2,theta2_2;
std::cout << status_1.format(CleanFmt) << std::endl << std::endl;
std::cout << status_2.format(CleanFmt) << std::endl << std::endl;
// 代价函数,输入变换后关节角度、之前的关节角度、权重,输出代价值
std::function<double (const Eigen::VectorXd,const Eigen::VectorXd,const Eigen::VectorXd)> cost_func
= [](const Eigen::VectorXd last_joint_status,
     const Eigen::VectorXd next_joint_status,
     const Eigen::VectorXd weight)->decltype(auto)
    {
    double cost_value = (next_joint_status - last_joint_status).array().square().matrix().transpose() * weight;
    std::cout<<"Cost value is "<<cost_value<<std::endl;
    return cost_value;
    };

// 求出关节角度后
return cost_func(last_joint_status,status_1,weight) > cost_func(last_joint_status,status_2,weight)?
       std::array<double,2>{theta1_1,theta2_1} : std::array<double,2>{theta1_2,theta2_2};
}

void detectCollisionSequence(std::array<double,2> last_joint_status,std::array<double,2> next_joint_status)
{

/* 
/ 两个一元二次项表示两个关节的时空状态
/ 关节位置的一阶导数在初始时间点以及最终时间点的值皆为0
/ 通过速度的积分就是关节变化量
*/ 

// 两个关节转动到目标位置所花时间
double total_spent_time;
// 第一个关节的方程参数
double a1,b1,c1,d1;
// 第二个关节的方程参数
double a2,b2,c2,d2;

// 
size_t count = 30;
double delta = total_spent_time / (float)count;
}

private:
fmp::EnvelopeGroupd envelope_group_;
fmp::JointLimitGroupd joint_limit_group_;

};

int main()
{
std::string envelope_config_file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/envelopes.yaml";
std::string joint_config_file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/joint_limits.yaml";

TwoLinkRobot::Pose target_pose;
target_pose<<1.0f,0.0f,0.0f,70.0f,
             0.0f,1.0f,0.0f,100.0f,
             0.0f,0.0f,1.0f,0.0f,
             0.0f,0.0f,0.0f,1.0f;

TwoLinkRobot singleRobot(joint_config_file,envelope_config_file);

std::array<double,2> solution = singleRobot.solveInverseTransform(target_pose);

return 0;
}