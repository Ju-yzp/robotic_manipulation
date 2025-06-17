#include<GL/gl.h>
#include<GLFW/glfw3.h>

#include<mujoco/mjdata.h>
#include<mujoco/mjrender.h>
#include<mujoco/mjvisualize.h>
#include<mujoco/mujoco.h>
#include<mujoco/mjmodel.h>

#include<inverse_transform/ur5_inverse_kinematics.h>
#include<inverse_transform/util.h>

#include<chrono>
#include<iostream>
#include<cmath>
#include<thread>

// mujoco资源
mjModel *model = nullptr;
mjData *data = nullptr;
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

void keyboard(GLFWwindow *window, int key, int scancdataode, int act,
              int mods) {
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
    mj_resetData(model, data);
    mj_forward(model, data);
  }
}

void mouse_button(GLFWwindow *window, int button, int act, int mods) {
  // update button state
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow *window, double xpos, double ypos) {
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
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
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

void scroll(GLFWwindow *window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

std::vector<float> get_sensor_data(const mjModel *model, const mjData *data,
                                   const std::string &sensor_name) {
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

Eigen::Vector2f generateCircle(float time,float radius,float x,float y,float angular_velocity)
{
  Eigen::Vector2f pos;
  pos(0) = x + radius * cos(2*M_PIf*time*angular_velocity);
  pos(1) = y + radius * sin(2*M_PIf*time*angular_velocity);

  return pos;
}

int failed_count,succeful_count;
int main()
{
    UR5RobotArm robot;

    char error_msg[1000] = "Could not load binary model !";
    std::string file_path = "/home/up/rm_mujoco_simulate/src/inverse_transform/scene.xml";
    //"/home/up/rm_mujoco_simulate/src/inverse_transform/scene.xml";
    model = mj_loadXML(file_path.c_str(),0,error_msg,1000);

    data = mj_makeData(model);

    if(!glfwInit())
    {
        mju_error("Could not initialize GLFW");
    }

    // 创建窗口以及初始化
    GLFWwindow *window = glfwCreateWindow(1200,900,"",NULL,NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 窗口绑定鼠标事件
    glfwSetKeyCallback(window,keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // 初始化可视化数据结构
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    mjv_makeScene(model,&scn,2000);
    mjr_makeContext(model,&con,mjFONTSCALE_150);

    robot.dh_table_[0][3] = 0.0f;
    robot.dh_table_[1][3] = -1.57f;
    robot.dh_table_[2][3] = 0.9f;
    robot.dh_table_[3][3] = 0.0f;
    robot.dh_table_[4][3] = 0.0f;
    robot.dh_table_[5][3] = 0.0f;
    mjtNum pos;// 记录关节角度用于更新运动学模型
    
    Solutions<6> solution;
    solution.theta[0] = 0.0f;
    solution.theta[1] = -1.5708f;
    solution.theta[2] = 0.9f;
    solution.theta[3] = 0.0f;
    solution.theta[4] = 0.0f;
    solution.theta[5] = 0.0f;
    Eigen::Matrix4f pose = forwardKinematics(robot.dh_table_);

    float radius = 35.0;
    float x = pose(0,3) - radius;
    float z = pose(2,3);
    float time_prev;
    float t_init = 1.0;

    while (!glfwWindowShouldClose(window)) {

          time_prev = data->time;

          while(data->time-time_prev < 1.0f/60.f){
          data->time +=0.02;
          //std::cout<<data->time<<std::endl;
          for(int i = 0; i < 6;i++)
          {
            data->qpos[i] = solution.theta[i];
          }

          mj_forward(model,data);

          if(data->time >= t_init){

          auto solutions = robot.inverseKinematics(pose);
          bool flag{false};

          for(auto &solution_:solutions){
          
            if(robot.isRightSolution(solution_,pose))
            {
              solution = solution_;
              solution.theta[4] *= -1.0f;
              flag = true;
              break;
            }
            }
            
            if(flag)
            {
              std::cout<<"规划成功 "<<std::endl;
              succeful_count++;
            }else{
              std::cout<<"规划失败 "<<std::endl;
              failed_count++;
            }
            std::cout<<" "<<std::endl;
            Eigen::Vector2f new_pos = generateCircle(data->time-t_init,radius,x,z,1.0);
            pose(0,3) = new_pos(0);
            pose(2,3) = new_pos(1);
          }

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);

        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
          }
        if(data->time > 15){
          break;
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        }
    }
    float num = (float)succeful_count /(failed_count+succeful_count)*100.0f;
    std::cout<<"规划成功率 "<<num<<"%"<<std::endl;
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    mj_deleteData(data);
    mj_deleteModel(model);

    return 0;
}

