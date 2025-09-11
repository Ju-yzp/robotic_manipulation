// motion_planning_tutorial
#include <motion_planning_tutorial/collision_detector.hpp>
#include <motion_planning_tutorial/controller.hpp>
#include <motion_planning_tutorial/planner.hpp>
#include <motion_planning_tutorial/problemDefinition.hpp>
#include <motion_planning_tutorial/robot_description.hpp>
#include <motion_planning_tutorial/ur5e_kinematic.hpp>

// mujoco
#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <cmath>
#include <iostream>
#include <thread>

namespace mujoco_resource {

// mujoco资源
mjModel* model = nullptr;
mjData* data = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// 将键盘行为与窗口事件进行关联
void keyboard(GLFWwindow* window, int key, int scancdataode, int act, int mods) {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        mj_resetData(model, data);
        mj_forward(model, data);
    }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift =
        (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
         glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(model, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

std::vector<float> get_sensor_data(
    const mjModel* model, const mjData* data, const std::string& sensor_name) {
    int sensor_id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
    if (sensor_id == -1) {
        std::cout << "no found sensor" << std::endl;
        return std::vector<float>();
    }
    int data_pos = model->sensor_adr[sensor_id];
    std::vector<float> sensor_data(model->sensor_dim[sensor_id]);
    for (int i = 0; i < sensor_data.size(); i++) {
        sensor_data[i] = data->sensordata[data_pos + i];
    }
    return sensor_data;
}
}  // namespace mujoco_resource

int main() {
    namespace mpt = motion_planning_tutorial;

    // UR5E运动学
    mpt::Ur5eParam a_table{0.0, 0.0, 425.0, 392.0, 0.0, 0.0};
    mpt::Ur5eParam d_table{163.0, 134.0, 0.0, 0.0, -100.0, 100.0};
    mpt::Ur5eParam alpha_table{0.0, -M_PI / 2.0, 0.0, 0.0, M_PI / 2.0, -M_PI / 2.0};
    mpt::Ur5eParam theta_table{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::shared_ptr<mpt::Ur5eKinematic> ur5e_kinematic =
        std::make_shared<mpt::Ur5eKinematic>(a_table, d_table, alpha_table, theta_table);

    ur5e_kinematic->set_id(0, "shoulder_pan");
    ur5e_kinematic->set_id(1, "shoulder_lift");
    ur5e_kinematic->set_id(2, "elbow");
    ur5e_kinematic->set_id(3, "wrist_1_link");
    ur5e_kinematic->set_id(4, "wrist_2_link");
    ur5e_kinematic->set_id(5, "wrist_3_link");

    // 先定义规划问题
    Eigen::Vector<double, 6> start_positions;
    start_positions << 1.0, -0.7, -1.2, 0.2, 0.2, 1.0;
    mpt::State start_state;
    start_state.positions = start_positions;
    mpt::State end_state;
    Eigen::Vector<double, 6> end_positions;
    end_positions << -2.0, 0, -1.6, 0.5, 0.8, 2.1;
    end_state.positions = end_positions;
    Eigen::Isometry3d goal_state = ur5e_kinematic->get_endeffector_pose(end_state);
    mpt::ProblemDefinition pd(start_state, goal_state);

    // 机械臂描述
    std::string configuration_file =
        "/home/up/robotics-manipulation/src/motion_planning_tutorial/config/ur5e_description.yaml";
    mpt::RobotDescription::SharedPtr robot_description =
        std::make_shared<mpt::RobotDescription>(configuration_file, ur5e_kinematic);

    // 碰撞检测器
    mpt::Scene scene;
    scene.obstacle_centers = {
        Eigen::Vector4d(-200.0, 500.0, 900.0, 1.0), Eigen::Vector4d(100.0, -400.0, 750.0, 1.0)
        // Eigen::Vector4d(300.0, 400.0, 100.0,1.0)
    };
    scene.obstacle_radius = {100.0, 100.0};
    std::shared_ptr<KD_TREE<pcl::PointXYZ>> kd_tree_;
    kd_tree_ = std::make_shared<KD_TREE<pcl::PointXYZ>>();
    mpt::CollisionDetector::UniquePtr collision_detector =
        std::make_unique<mpt::CollisionDetector>(kd_tree_);

    collision_detector->set_scene(scene);

    // 规划器
    mpt::Planner planner(robot_description, ur5e_kinematic, collision_detector);
    planner.set_id_and_name(0, "shoulder_pan");
    planner.set_id_and_name(1, "shoulder_lift");
    planner.set_id_and_name(2, "elbow");
    planner.set_id_and_name(3, "wrist_1_link");
    planner.set_id_and_name(4, "wrist_2_link");
    planner.set_id_and_name(5, "wrist_3_link");

    planner.solve(pd);

    if (pd.get_state())
        std::cout << "Succeful to solve the planning problem." << std::endl;
    else {
        std::cout << "Failed to solve the planning problem." << std::endl;
        return -1;
    }

    auto path = pd.get_initial_path();
    for (const auto& state : path) {
        std::cout << "State: ";
        for (const auto& pos : state.positions) {
            std::cout << pos << " ";
        }
        std::cout << std::endl;
    }

    mpt::Controller controller(robot_description);
    controller.set_id_and_name(0, "shoulder_pan");
    controller.set_id_and_name(1, "shoulder_lift");
    controller.set_id_and_name(2, "elbow");
    controller.set_id_and_name(3, "wrist_1_link");
    controller.set_id_and_name(4, "wrist_2_link");
    controller.set_id_and_name(5, "wrist_3_link");

    auto timepoint = controller.set_initial_time_point(pd);
    auto bspilne = controller.smoothPath(pd, timepoint, mpt::SmoothType::BASIC_SPLINE);

    std::string file_path = "/home/up/mujoco_resource/scene.xml";
    char error_msg[1000] = "Could not load binary model !";
    mujoco_resource::model = mj_loadXML(file_path.c_str(), 0, error_msg, 1000);

    mujoco_resource::data = mj_makeData(mujoco_resource::model);

    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // 创建窗口以及初始化
    GLFWwindow* window = glfwCreateWindow(1200, 900, "", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 窗口绑定鼠标事件
    glfwSetKeyCallback(window, mujoco_resource::keyboard);
    glfwSetCursorPosCallback(window, mujoco_resource::mouse_move);
    glfwSetMouseButtonCallback(window, mujoco_resource::mouse_button);
    glfwSetScrollCallback(window, mujoco_resource::scroll);

    // 初始化可视化数据结构
    mjv_defaultCamera(&mujoco_resource::cam);
    mjv_defaultOption(&mujoco_resource::opt);
    mjv_defaultScene(&mujoco_resource::scn);
    mjr_defaultContext(&mujoco_resource::con);

    mjv_makeScene(mujoco_resource::model, &mujoco_resource::scn, 2000);
    mjr_makeContext(mujoco_resource::model, &mujoco_resource::con, mjFONTSCALE_150);

    int count = 0;
    double time_sum = bspilne.getTimeSum();
    double time_step = time_sum / double(300);

    while (!glfwWindowShouldClose(window) && count <= 300) {
        auto positions = bspilne.evaluateDeBoor(time_step * (double)count);
        mujoco_resource::data->qpos[0] = positions(0);
        mujoco_resource::data->qpos[1] = positions(1);
        mujoco_resource::data->qpos[2] = positions(2);
        mujoco_resource::data->qpos[3] = positions(3);
        mujoco_resource::data->qpos[4] = -positions(4);
        mujoco_resource::data->qpos[5] = positions(5);
        mj_forward(mujoco_resource::model, mujoco_resource::data);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(
            mujoco_resource::model, mujoco_resource::data, &mujoco_resource::opt, NULL,
            &mujoco_resource::cam, mjCAT_ALL, &mujoco_resource::scn);
        mjr_render(viewport, &mujoco_resource::scn, &mujoco_resource::con);
        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        count++;
    }

    mjv_freeScene(&mujoco_resource::scn);
    mjr_freeContext(&mujoco_resource::con);

    mj_deleteData(mujoco_resource::data);
    mj_deleteModel(mujoco_resource::model);

    return 0;
}
