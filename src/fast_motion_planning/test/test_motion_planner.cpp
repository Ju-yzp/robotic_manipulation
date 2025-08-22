// fast motion planning
#include<fast_motion_planning/sampler.hpp>
#include<fast_motion_planning/plan_problem.hpp>
#include<fast_motion_planning/collision_detector.hpp>
#include<fast_motion_planning/ur5e_kinematic_solver.hpp>
#include<fast_motion_planning/mujoco_window.hpp>

// cpp
#include<memory>
#include<thread>
#include<chrono>

double to(double value)
{double new_value = std::fmod(value,2.0f*M_PIf);
return new_value = new_value < -M_PIf ? new_value + 2.0f * M_PIf : 
            ((new_value > M_PIf) ? M_PIf - new_value : new_value);

}
int main()
{
namespace fmp = fast_motion_planning;

// 机器人描述(包络体和关节限制)
fmp::RobotDescription<double>::SharedPtr ur5e_description = std::make_shared<fmp::RobotDescription<double>>();
std::string file = "/home/up/robotics-manipulation/src/fast_motion_planning/config/ur5e_description.yaml";
ur5e_description->parse_configuration_file(file);

// 运动学解算器
Ur5eKinematicSolver::Ur5eParam a_table{0.0,0.0,425.0,392.0,0.0,0.0};
Ur5eKinematicSolver::Ur5eParam d_table{163.0,134.0,0.0,0.0,-100.0,100.0};
Ur5eKinematicSolver::Ur5eParam alpha_table{0.0,-M_PI/2.0,0.0,0.0,M_PI/2.0,-M_PI/2.0};
std::shared_ptr<Ur5eKinematicSolver> ur5e_kinematic_solver = std::make_shared<Ur5eKinematicSolver>(a_table,d_table,alpha_table,ur5e_description);

// 设置名称映射
ur5e_kinematic_solver->set_joint_name(0, "shoulder_pan");
ur5e_kinematic_solver->set_joint_name(1, "shoulder_lift");
ur5e_kinematic_solver->set_joint_name(2, "elbow");
ur5e_kinematic_solver->set_joint_name(3, "wrist_1_link");
ur5e_kinematic_solver->set_joint_name(4, "wrist_2_link");
ur5e_kinematic_solver->set_joint_name(5, "wrist_3_link");

// 规划问题
Ur5eKinematicSolver::Ur5eParam start_thetas{1.0,-0.7,-1.2,0.2,0.2,1.0};
Ur5eKinematicSolver::Ur5eParam goal_thetas{-1.3,-0.1,-0.1,0.9,0.2,0.2};
Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
std::cout << "----Goal Endeffector Pose----" << std::endl;
std::cout << ur5e_kinematic_solver->get_endeffector_pose(start_thetas).format(CleanFmt) << std::endl;
std::cout << ur5e_kinematic_solver->get_endeffector_pose(goal_thetas).format(CleanFmt) << std::endl;
ur5e_kinematic_solver->update_envelopes_position(start_thetas);


Eigen::Matrix4d target_pose;

target_pose << 1.0f,0.0f,0.0f,100.0f,
               0.0f,1.0f,0.0f,-100.0f,
               0.0f,0.0f,1.0f,500.0f,
               0.0f,0.0f,0.0f,1.0f;
fmp::PlanProblem<double> problem(ur5e_kinematic_solver->get_endeffector_pose(start_thetas),
      target_pose);

// 规划场景
fmp::Scene scene;
// 设置障碍物
scene.obstacles_position.emplace_back(Eigen::Vector4d{-200.0,500.0,900.0,1.0});
scene.radius.emplace_back(100.0);
scene.obstacles_position.emplace_back(Eigen::Vector4d{100.0,100.0,700.0,1.0});
scene.radius.emplace_back(100.0);
scene.obstacles_position.emplace_back(Eigen::Vector4d{-300.0,00.0,300.0,1.0});
scene.radius.emplace_back(100.0);
// 碰撞检测器
std::unique_ptr<fmp::CollisionDetector> collision_detector = std::make_unique<fmp::CollisionDetector>(ur5e_description,scene);
collision_detector->set_name(0, "shoulder_pan");
collision_detector->set_name(1, "shoulder_lift");
collision_detector->set_name(2, "elbow");
collision_detector->set_name(3, "wrist_1_link");
collision_detector->set_name(4, "wrist_2_link");
collision_detector->set_name(5, "wrist_3_link");

// if(collision_detector->is_collision_with_obstacles())
//   std::cout<<"No"<<std::endl;
// 状态空间管理器
std::unique_ptr<fmp::StatusSpaceManager> status_space_manager = std::make_unique<fmp::StatusSpaceManager>(-900.0,900.0,-900.0,900.0,0.0,850.0);

// 采样器（其实行为更像规划器）
std::unique_ptr<fmp::Sampler> sampler = std::make_unique<fmp::Sampler>(status_space_manager,problem,collision_detector,
                                                                       std::dynamic_pointer_cast<fmp::KinematicSolverBaseInterface>(ur5e_kinematic_solver),40.0,80000);


//权重函数
fmp::Sampler::WeightFunc weight_func = [](std::vector<Eigen::VectorXd>& solutions,const Eigen::VectorXd& last_solution)->decltype(auto)
{
static Eigen::VectorXd weight = Eigen::Vector<double,6>{0.4,0.1,0.2,0.2,0.1,0.0};
std::sort(solutions.begin(),solutions.end(),[=](const Eigen::VectorXd& a,const Eigen::VectorXd& b)
{return ((a - last_solution).transpose() * weight) < ((b - last_solution).transpose() * weight);});
return solutions[0];};

sampler->bindWeightFunction(weight_func);

sampler->plan();

auto path = sampler->get_path();

if(path.empty())
   return 0;

std::cout<<(int)path.size()<<std::endl;

for(auto& point:path)
{
point( 0) = to(point(0));
point( 1) = to(point(1));
point( 2) = to(point(2));
point( 3) = to(point(3));
point( 4) = to(point(4));
point( 5) = to(point(5));
}
std::vector<Eigen::VectorXd> new_path;
Eigen::VectorXd last_status;
size_t iter{0};
for(auto solution:path)
{
std::cout<<solution[0]<<std::endl;
if(iter == 0)
{
new_path.push_back(solution);
}
else{
float count = 20;
if(std::abs(last_status[0] - solution[0]) > 1.0f)
   count = 100;
for(int i = 1; i <= count;i++)
{
new_path.push_back(
Eigen::Vector<double,6>{
last_status[0] + (solution[0] - last_status[0] ) * (i/count),
last_status[1] + (solution[1] - last_status[1] ) * (i/count),
last_status[2] + (solution[2] - last_status[2] ) * (i/count),
last_status[3] + (solution[3] - last_status[3] ) * (i/count),
(last_status[4] + (solution[4] - last_status[4] ) * (i/count)),
(last_status[5] + (solution[5] - last_status[5] ) * (i/count)),
}
);
}
}
last_status = solution;
iter++;
}

char error_msg[1000] = "Could not load binary model !";
std::string file_path = "/home/up/mujoco_resource/scene.xml";

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

mujoco_resource::data->qpos[0] = 1.0f;
mujoco_resource::data->qpos[1] = -0.7f;
mujoco_resource::data->qpos[2] = -1.2f;
mujoco_resource::data->qpos[3] = 0.2f;
mujoco_resource::data->qpos[4] = -0.2f;
mujoco_resource::data->qpos[5] = 1.0f;

bool flag{true};

while (!glfwWindowShouldClose(window) && flag) {
    for(auto solution:new_path)
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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    flag = false;
}
mjv_freeScene(&mujoco_resource::scn);
mjr_freeContext(&mujoco_resource::con);

mj_deleteData(mujoco_resource::data);
mj_deleteModel(mujoco_resource::model);

return 0;
}