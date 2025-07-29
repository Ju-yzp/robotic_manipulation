// ros2
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

// 轨迹信息
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

// 规划控制
#include <motion_planning/inverse_kinematic_solver.hpp>
#include <motion_planning/robotModel.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("send_trajectory");
  auto pub = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/ur5e_controller/joint_trajectory", 10);

  // get robot description
  // auto robot_param = rclcpp::Parameter();
  // node->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING);
  // node->get_parameter("robot_description", robot_param);
  // auto robot_description = robot_param.as_string();

  // 给关节填入信息
  trajectory_msgs::msg::JointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = node->now();
  trajectory_msg.joint_names.push_back("shoulder_pan_joint");
  trajectory_msg.joint_names.push_back("shoulder_lift_joint");
  trajectory_msg.joint_names.push_back("elbow_joint");
  trajectory_msg.joint_names.push_back("wrist_1_joint");
  trajectory_msg.joint_names.push_back("wrist_2_joint");
  trajectory_msg.joint_names.push_back("wrist_3_joint");

  // ur5e有六个关节
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_point_msg;
  trajectory_point_msg.positions.resize(6);
  trajectory_point_msg.velocities.resize(6);

  // 减少命名空间的部分
  using namespace motion_planning;

  // ur5e的DH参数，采取改进法进行DH建模
  typedef std::array<float,UR5E_DOF> Ur5Param;

  // 以弧度制和mm作为基本单位
  Ur5Param a{0.0f,0.0f,425.0f,392.0f,0.0f,0.0f};
  Ur5Param d{163.0f,134.0f,0.0f,0.0f,-100.0f,100.0f};
  Ur5Param alpha{0.0f,-M_PIf/2.0f,0.0f,0.0f,M_PIf/2.0f,-M_PIf/2.0f};
  Ur5Param theta{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

  // ur5e的关节限位
  Ur5Param joint_limit_upper{0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
  Ur5Param joint_limit_lower{0.0f,0.0f,0.0f,0.0f,0.6f,0.8f};
  JointLimit<UR5E_DOF> joint_limit{joint_limit_upper,joint_limit_lower};

  // ur5e的机器臂模型
  std::shared_ptr<RobotModel<UR5E_DOF>> robot_model = std::make_shared<RobotModel<UR5E_DOF>>(a,d,alpha,theta,joint_limit);

  // 将机械臂模型与逆运动学求解器关联
  std::unique_ptr<InverseKinematicSolver> inverse_kinematic_solver = std::make_unique<InverseKinematicSolver>(robot_model);

  Eigen::Matrix4f endeffector_pose;
  endeffector_pose<<1.0f,0.0f,0.0f,300.0f,
                    0.0f,-1.0f,0.0f,100.0f,
                    0.0f,0.0f,-1.0f,300.0f,
                    0.0f,0.0f,0.0f,1.0f;
      
  auto solutions = inverse_kinematic_solver->inverseKinematic(endeffector_pose);
  if(!solutions.solutions_.empty())
  {
  std::cout<<"Succeful to get correct solution to make robot arm reach specified pose"<<std::endl;
  for(size_t index{0}; index < UR5E_DOF; index++)
  {
  std::cout<<"The "<<index<<" value is"<<solutions.solutions_[0][index]<<std::endl;
  if(index == 4)
    robot_model->set_theta(-solutions.solutions_[0][index], index);
  else 
    robot_model->set_theta(solutions.solutions_[0][index], index);
  }
  }
  else
    std::cout<<"Failed to get correct solution to make robot arm reach specified pose"<<std::endl;
  double total_time = 3.0; // 总共花费时间
  int trajectory_len = 200; // 轨迹长度
  double dt = total_time / static_cast<double>(trajectory_len - 1); // 两个轨迹点之间时间间隔

  for (int i = 0; i < trajectory_len; i++)
  {
    // set endpoint twist
    double t = i / (static_cast<double>(trajectory_len - 1));
    Eigen::Vector<float,3> twist_velocity;
    twist_velocity(0) = 2 * 0.3 * cos(2 * M_PI * t) * 1000.0f;
    twist_velocity(1) = -0.3 * sin(2 * M_PI * t) * 1000.0f;
    twist_velocity(2) = 0.0f;

    Eigen::VectorXf joint_velocity = inverse_kinematic_solver->get_joint_velocity(twist_velocity);

    // 设置关节速度和位置
    auto joint_position =  trajectory_point_msg.positions.data();
    auto joint_velocities = trajectory_point_msg.velocities.data();

    for(size_t index = 0; index < UR5E_DOF; index++)
    {
      if(index == 4){
        joint_velocities[index] = -joint_velocity(index);
        joint_position[index] = -robot_model->get_theta(index) + joint_velocities[index] * dt;
        std::cout<<"Theta "<<index<<" value is "<<joint_position[index]<<std::endl;
        robot_model->set_theta(-joint_position[index], index);}
      else{
        joint_velocities[index] = joint_velocity(index);
        joint_position[index] = robot_model->get_theta(index) + joint_velocities[index] * dt;
        std::cout<<"Theta "<<index<<" value is "<<joint_position[index]<<std::endl;
        robot_model->set_theta(joint_position[index], index);}  
    }

    // 设置时间信息
    double time_point = total_time * t;
    double time_point_sec = std::floor(time_point);

    // 持续时间
    trajectory_point_msg.time_from_start.sec = static_cast<int>(time_point_sec);
    trajectory_point_msg.time_from_start.nanosec =
      static_cast<int>((time_point - time_point_sec) * 1E9);
    // 把轨迹点填入轨迹消息体
    trajectory_msg.points.push_back(trajectory_point_msg);
  }

  // 最后填入速度为0的轨迹点消息
  auto & last_point_msg = trajectory_msg.points.back();
  std::fill(last_point_msg.velocities.begin(), last_point_msg.velocities.end(), 0.0);

  pub->publish(trajectory_msg);
  while (rclcpp::ok())
  {
    // 休眠一段时间，避免空转过度占用cpu
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  return 0;
}