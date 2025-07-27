#include <cstddef>

#include <memory>
#include <array>

#include <iostream>
#include <cmath>

#include <motion_planning/mujoco_window.hpp>
#include <motion_planning/inverse_kinematic_solver.hpp>
#include <motion_planning/robotModel.hpp>

int main()
{
// 减少命名空间的部分
using namespace motion_planning;

// ur5e的DH参数，采取改进法进行DH建模
typedef std::array<float,UR5E_DOF> Ur5Param;

// 以弧度制和mm作为基本单位
Ur5Param a{0.0f,0.0f,425.0f,392.0f,0.0f,0.0f};
Ur5Param d{163.0f,134.0f,0.0f,0.0f,-100.0f,100.0f};
Ur5Param alpha{0.0f,-M_PIf/2.0f,0.0f,0.0f,M_PIf/2.0f,-M_PIf/2.0f};
Ur5Param theta{0.4f,2.0f,0.8f,1.0f,0.0f,1.0f};

// ur5e的关节限位
Ur5Param joint_limit_upper{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
Ur5Param joint_limit_lower{0.0f,0.0f,0.0f,0.0f,0.6f,0.8f};
JointLimit<UR5E_DOF> joint_limit{joint_limit_upper,joint_limit_upper};

// ur5e的机器臂模型
std::shared_ptr<RobotModel<UR5E_DOF>> robot_model = std::make_shared<RobotModel<UR5E_DOF>>(a,d,alpha,theta,joint_limit);
Eigen::Matrix4f origin_pose = robot_model->get_endeffector_status();

// 将机械臂模型与逆运动学求解器关联
std::unique_ptr<InverseKinematicSolver> inverse_kinematic_solver = std::make_unique<InverseKinematicSolver>(robot_model);

Eigen::Matrix4f target_pose;

target_pose << 1.0f,0.0f,0.0f,100.0f,
               0.0f,1.0f,0.0f,200.0f,
               0.0f,0.0f,1.0f,700.0f,
               0.0f,0.0f,0.0f,1.0f;


//target_pose = target_pose * endeffector_to_frame6.inverse();           
// Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
// std::cout << target_pose.format(CleanFmt) << std::endl << std::endl;
Solutions<UR5E_DOF> solutions = inverse_kinematic_solver->inverseKinematic(target_pose);

robot_model->selectSolutions(solutions,target_pose);

std::cout<<"Size of solutions is "<<(int)solutions.solutions_.size()<<std::endl;


// 加载模型资源
char error_msg[1000] = "Could not load binary model !";
std::string file_path = "/home/up/robotics-manipulation/src/mujoco_resource/scene.xml";

mujoco_resource::model = mj_loadXML(file_path.c_str(),0,error_msg,1000);

mujoco_resource::data = mj_makeData(mujoco_resource::model);

if(!glfwInit())
{
    mju_error("Could not initialize GLFW");
}

// 创建窗口以及初始化
GLFWwindow *window = glfwCreateWindow(1200,900,"",NULL,NULL);
glfwMakeContextCurrent(window);
glfwSwapInterval(1);

// 窗口绑定鼠标事件
glfwSetKeyCallback(window,mujoco_resource::keyboard);
glfwSetCursorPosCallback(window, mujoco_resource::mouse_move);
glfwSetMouseButtonCallback(window, mujoco_resource::mouse_button);
glfwSetScrollCallback(window, mujoco_resource::scroll);

// 初始化可视化数据结构
mjv_defaultCamera(&mujoco_resource::cam);
mjv_defaultOption(&mujoco_resource::opt);
mjv_defaultScene(&mujoco_resource::scn);
mjr_defaultContext(&mujoco_resource::con);

mjv_makeScene(mujoco_resource::model,&mujoco_resource::scn,2000);
mjr_makeContext(mujoco_resource::model,&mujoco_resource::con,mjFONTSCALE_150);

mjtNum pos;// 记录关节角度用于更新运动学模型

if(!solutions.solutions_.empty())
{
    mujoco_resource::data->qpos[0] = solutions.solutions_[3][0];
    mujoco_resource::data->qpos[1] = solutions.solutions_[3][1];
    mujoco_resource::data->qpos[2] = solutions.solutions_[3][2];
    mujoco_resource::data->qpos[3] = solutions.solutions_[3][3];
    mujoco_resource::data->qpos[4] = -solutions.solutions_[3][4];
    mujoco_resource::data->qpos[5] = solutions.solutions_[3][5];
    std::cout<<"assign the first solution default solution"<<std::endl;
}
else 
    std::cout<<"Failed to find a vaild solution that make robot arm reach the target pose"<<std::endl;

while (!glfwWindowShouldClose(window)) {
    mj_forward(mujoco_resource::model,mujoco_resource::data);

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(mujoco_resource::model, mujoco_resource::data, &mujoco_resource::opt, NULL, &mujoco_resource::cam, mjCAT_ALL, &mujoco_resource::scn);

    mjr_render(viewport, &mujoco_resource::scn, &mujoco_resource::con);

    glfwSwapBuffers(window);
    glfwPollEvents();
}
mjv_freeScene(&mujoco_resource::scn);
mjr_freeContext(&mujoco_resource::con);

mj_deleteData(mujoco_resource::data);
mj_deleteModel(mujoco_resource::model);

return 0;

} 