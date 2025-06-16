# mujoco_sim_practise

## Consist of project

1.inverse_transform:
ur5e机械臂的逆运动学解算部分。

2.path_planing:
路径规划器模块，通过RRT*等算法生成一条安全路径，避免与周围障碍物产生碰撞。

3.robot_arm_controller:
通过自定义的PID(或者MPC)控制器，实现对仿真电机的控制。

## Insight

在未来如果选择与一些语义分析模型结合，那么将会增加应用场景。通过分析当前执行的任务，可以知道对于发生碰撞的容忍度是多少，
进而动态设置路径规划模块参数，更加智能化地服务人类。