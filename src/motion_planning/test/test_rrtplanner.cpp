#include <cstddef>
#include <memory>

#include <motion_planning/RRTPlanner.hpp>
#include <motion_planning/inverse_kinematic_solver.hpp>
#include <motion_planning/scence.hpp>
#include <motion_planning/robotModel.hpp>
#include <motion_planning/mujoco_window.hpp>
#include <motion_planning/RRTPlanner.hpp>

int main()
{

// 减少命名空间的部分
using namespace motion_planning;

// ur5e的DH参数，采取改进法进行DH建模
typedef std::array<float,UR5E_DOF> Ur5Param;

// 以弧度制和mm作为基本单位
Ur5Param a{0.0f,0.0f,425.0f,392.0f,0.0f,0.0f};
Ur5Param d{163.0f,134.0f,0.0f,0.0f,-100.0f,0.0f};
Ur5Param alpha{0.0f,-M_PIf/2.0f,0.0f,0.0f,M_PIf/2.0f,-M_PIf/2.0f};
Ur5Param theta{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

// ur5e的关节限位
Ur5Param joint_limit_upper{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
Ur5Param joint_limit_lower{0.0f,0.0f,0.0f,0.0f,0.6f,0.8f};
JointLimit<UR5E_DOF> joint_limit{joint_limit_upper,joint_limit_upper};

// ur5e的机器臂模型
std::shared_ptr<RobotModel<UR5E_DOF>> robot_model = std::make_shared<RobotModel<UR5E_DOF>>(a,d,alpha,theta,joint_limit);

// 将机械臂模型与逆运动学求解器关联
std::shared_ptr<InverseKinematicSolver> inverse_kinematic_solver = std::make_shared<InverseKinematicSolver>(robot_model);  

// 使用场景
Scence scence;

// 添加障碍物
std::shared_ptr<SphereObstacle> obstacble_1 = std::make_shared<SphereObstacle>();
obstacble_1->radius = 60.0f;
obstacble_1->x = -200.0f;
obstacble_1->y = 500.0f;
obstacble_1->z = 900.0f;
scence.add_obstacle(obstacble_1);

std::shared_ptr<SphereObstacle> obstacble_2 = std::make_shared<SphereObstacle>();
obstacble_2->radius = 60.0f;
obstacble_2->x = 100.0f;
obstacble_2->y = 100.0f;
obstacble_2->z = 700.0f;
scence.add_obstacle(obstacble_2);

// 添加包络体
SperhreEnvelope sperhre_envelope;
sperhre_envelope.radius = 92.0f;
sperhre_envelope.x = 0.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 0.0f;
robot_model->add_envelope(sperhre_envelope,0);

// 1
sperhre_envelope.radius = 92.0f;
sperhre_envelope.x = 280.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 4.0f;
robot_model->add_envelope(sperhre_envelope,1);

sperhre_envelope.radius = 92.0f;
sperhre_envelope.x = 420.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 4.0f;
robot_model->add_envelope(sperhre_envelope,1);

sperhre_envelope.radius = 92.0f;
sperhre_envelope.x = 140.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 4.0f;
robot_model->add_envelope(sperhre_envelope,1);

sperhre_envelope.radius = 92.0f;
sperhre_envelope.x = 0.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 4.0f;
robot_model->add_envelope(sperhre_envelope,1);

// 2
sperhre_envelope.radius = 88.46f;
sperhre_envelope.x = 249.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = -115.0f;
robot_model->add_envelope(sperhre_envelope,2);

sperhre_envelope.radius = 88.46f;
sperhre_envelope.x = 119.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = -115.0f;
robot_model->add_envelope(sperhre_envelope,2);

sperhre_envelope.radius = 88.46f;
sperhre_envelope.x = 379.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = -115.0f;
robot_model->add_envelope(sperhre_envelope,2);

// 3
sperhre_envelope.radius = 75.0f;
sperhre_envelope.x = 0.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = -10.340f;
robot_model->add_envelope(sperhre_envelope,3);

// 4
sperhre_envelope.radius = 70.0f;
sperhre_envelope.x = 0.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 91.0f;
robot_model->add_envelope(sperhre_envelope,4);

// 5
sperhre_envelope.radius = 60.0f;
sperhre_envelope.x = 0.0f;
sperhre_envelope.y = 0.0f;
sperhre_envelope.z = 85.6f;
robot_model->add_envelope(sperhre_envelope,5);

//设置机械臂状态
robot_model->set_theta(1.0f,0);
robot_model->set_theta(-0.7f,1);
robot_model->set_theta(-1.2f,2);
robot_model->set_theta(0.2f,3);
robot_model->set_theta(0.2f,4);
robot_model->set_theta(1.0f,5);

//测试检测碰撞模块
if(robot_model->isOccurCollision(scence))
   std::cout<<"Occur collision between obstacles with robot arm"<<std::endl;

std::unique_ptr<RRTPlanner<UR5E_DOF>> rrt_planner = std::make_unique<RRTPlanner<UR5E_DOF>>(
dynamic_pointer_cast<InverseKinematicBaseInterface<UR5E_DOF>>(inverse_kinematic_solver),robot_model);

Eigen::Matrix4f target_pose;

target_pose << 1.0f,0.0f,0.0f,100.0f,
               0.0f,1.0f,0.0f,-100.0f,
               0.0f,0.0f,1.0f,500.0f,
               0.0f,0.0f,0.0f,1.0f;


Eigen::Matrix4f endeffector_to_frame6;

endeffector_to_frame6 << 1.0f,0.0f,0.0f,0.0f,
                         0.0f,1.0f,0.0f,0.0f,
                         0.0f,0.0f,1.0f,100.0f,
                         0.0f,0.0f,0.0f,1.0f;

target_pose = target_pose * endeffector_to_frame6.inverse(); 

Eigen::Matrix4f source_pose;
source_pose = robot_model->get_endeffector_status();

Eigen::Matrix4f p = source_pose  * endeffector_to_frame6;
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << source_pose.format(CleanFmt) << std::endl << std::endl;

auto solutions = inverse_kinematic_solver->inverseKinematic(target_pose);

// 路径规划
Trajectory<UR5E_DOF> tarjectoies = rrt_planner->plan(scence,target_pose,source_pose);
if(tarjectoies.plan_tarjectory.empty())
   std::cout<<"Failed to serach for a safety path from source pose to target pose"<<std::endl;
else 
   std::cout<<"Succefull to serach for a safety path from source pose to target pose"<<std::endl;

// 加载模型资源
char error_msg[1000] = "Could not load binary model !";
std::string file_path = "/home/up/robotics-manipulation/src/mujoco_resource/ur5e.xml";

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
std::cout<<"wow"<<std::endl;
mjtNum pos;// 记录关节角度用于更新运动学模型

mujoco_resource::data->qpos[0] = 1.0f;
mujoco_resource::data->qpos[1] = -0.7f;
mujoco_resource::data->qpos[2] = -1.2f;
mujoco_resource::data->qpos[3] = 0.2f;
mujoco_resource::data->qpos[4] = -0.2f;
mujoco_resource::data->qpos[5] = 1.0f;
// if(!solutions.solutions_.empty())
// {
//     mujoco_resource::data->qpos[0] = solutions.solutions_[0][0];
//     mujoco_resource::data->qpos[1] = solutions.solutions_[0][1];
//     mujoco_resource::data->qpos[2] = solutions.solutions_[0][2];
//     mujoco_resource::data->qpos[3] = solutions.solutions_[0][3];
//     mujoco_resource::data->qpos[4] = -solutions.solutions_[0][4];
//     mujoco_resource::data->qpos[5] = solutions.solutions_[0][5];
//     std::cout<<"assign the first solution default solution"<<std::endl;
// }
// else 
//     std::cout<<"Failed to find a vaild solution that make robot arm reach the target pose"<<std::endl;

// mujoco_resource::data->qpos[0] = 1.0f;
// mujoco_resource::data->qpos[1] = -0.7f;
// mujoco_resource::data->qpos[2] = -1.2f;
// mujoco_resource::data->qpos[3] = 0.2f;
// mujoco_resource::data->qpos[4] = -0.2f;
// mujoco_resource::data->qpos[5] = 1.0f;

while (!glfwWindowShouldClose(window)) {
    for(auto solution:tarjectoies.plan_tarjectory)
    {
    mujoco_resource::data->qpos[0] = solution[0];
    mujoco_resource::data->qpos[1] = solution[1];
    mujoco_resource::data->qpos[2] = solution[2];
    mujoco_resource::data->qpos[3] = solution[3];
    mujoco_resource::data->qpos[4] = -solution[4];
    mujoco_resource::data->qpos[5] = solution[5];
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
}
mjv_freeScene(&mujoco_resource::scn);
mjr_freeContext(&mujoco_resource::con);

mj_deleteData(mujoco_resource::data);
mj_deleteModel(mujoco_resource::model);

return 0;

}