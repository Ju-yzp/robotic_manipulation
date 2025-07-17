# Motion_Planning


## Quick Start

```sh
# 配置环境，Eigen,Glfw,mujoco,前提是你使用镜像或者科学上网
sudo update
sudo apt install libeigen3-dev
sudo apt install libglfw3-dev

# mujoco部分在官方部分查看下载方法

# 编译项目
mkdir -p build && cd build
cmake ..
make -j<core num> # 选择的编译线程数目
./test_inverse_transform # 运行逆运动学部分demo
```

![注意徽章](https://custom-icon-badges.demolab.com/badge/注意-blue.svg?logo=octicon-alert)
```sh
# 修改test_inverse_tranform.cpp的第65行，改为文件在自己电脑文件系统中的绝对路径
# 加载模型资源
char error_msg[1000] = "Could not load binary model !";
std::string file_path = "/home/up/robotics-manipulation/src/mujoco_resource/scene.xml";
```

## Display

长方体是代表位姿

![](img/demo.gif)