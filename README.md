# Mujoco_Simulate_Practise

## Features

- 通过实现八叉树(邻域快速搜索特性)，建立机械臂周围环境，后面做一版过滤机械臂本体，应该有很多种方案可以选择
- 通过rrt实现路径规划，轨迹规划部分由电机控制器实现
- 根据UR5E的解析解的有限性，可以通过检测每一个解对应的机械臂包络体集是否与障碍物发生碰撞，选择有效解

## Quick Start

```sh
# 配置环境:mujoco,glfw,eigen
# 等我到时候写一个脚本吧，自动检测计算机系统已经存在的包
# 如果包存在，但是版本不兼容就会自动更新包，然后安装缺失的包

mkdir build && cd build
cmake ..
make -j<num> #根据自己实际核心数目选择并行编译线程数量，我的项目也不是很大
```

## Outlook

- 通过自己实现的Scence类，设计神经网络对周围物体进行分类，而不是只是被简单定义为障碍物与被抓取物，周围物体应该拥有更多的信息

- 由于作者还处于大学二年级阶段，所以对于软件设计，机械臂，数学，电机控制等方面不是很深入，所以后面会慢慢改进本项目

## Contact Way

- 📧 邮箱：Jup230551l@outlook.com
- 🔗 GitHub：[![GitHub](https://img.shields.io/badge/GitHub-Ju-yzp)](https://github.com/Ju-yzp)

