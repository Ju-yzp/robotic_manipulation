cmake_minimum_required(VERSION 3.22.0)
project(
  fast_motion_planning
  VERSION 0.1.0
  DESCRIPTION "A custom library for problem of motion planing"
  LANGUAGES CXX)

# 如果用户没有设置构建类型，那么默认设置为Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)

# ros2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(Boost REQUIRED)

# 可视化库visualization_tools
add_library(visualization_tools src/trajectory_visualization.cpp)

target_include_directories(
  visualization_tools
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include> ${urdf_INCLUDE_DIRS}
         ${tf2_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}
         ${visualization_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS})

target_link_libraries(
  visualization_tools
  PUBLIC ${urdf_LIBRARIES} ${tf2_LIBRARIES} ${visualization_msgs_LIBRARIES}
         ${rclcpp_LIBRARIES} Eigen3::Eigen)

# 机器人参数库robot_params(加速度约束，包络体，速度约束，关节位置约束)
add_library(robot_params src/RobotParams.cpp)

target_include_directories(
  robot_params PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                      ${Eigen3_INCLUDE_DIRS})

target_link_libraries(robot_params PUBLIC Eigen3::Eigen yaml-cpp)

# 测试解析配置文件
add_executable(test_config_parser test/test_config_parser.cpp)

target_include_directories(
  test_config_parser PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

target_link_libraries(test_config_parser robot_params)

# 测试ur5e运动学解算器
add_executable(test_ur5e_kinematic_solver src/ur5e_kinematic.cpp
                                          test/test_ur5e_kinematic_solver.cpp)

target_include_directories(
  test_ur5e_kinematic_solver
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         ${Eigen3_INCLUDE_DIRS})

target_link_libraries(test_ur5e_kinematic_solver Eigen3::Eigen)

# 测试采样器
add_executable(test_sampler src/ur5e_kinematic.cpp src/sampler.cpp
                            test/test_sampler.cpp)

target_include_directories(
  test_sampler PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                      ${Eigen3_INCLUDE_DIRS})

target_link_libraries(test_sampler Eigen3::Eigen robot_params)

# 测试最邻近搜索器
add_executable(test_nearestNeighbors src/ur5e_kinematic.cpp src/sampler.cpp
                                     test/test_nearestNeighbors.cpp)

target_include_directories(
  test_nearestNeighbors PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                               ${Eigen3_INCLUDE_DIRS})

target_link_libraries(test_nearestNeighbors Eigen3::Eigen robot_params)

# 测试B样条，使用pybind11结合python进行可视化处理
find_package(Python COMPONENTS Interpreter Development)
find_package(pybind11 CONFIG)

# 测试规划器
add_executable(
  test_planner
  src/ur5e_kinematic.cpp
  src/sampler.cpp
  src/controller.cpp
  src/collision_detector.cpp
  src/non_uniform_bspline.cpp
  src/rrt_planner.cpp
  test/test_planner.cpp)

target_include_directories(
  test_planner PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                      ${Eigen3_INCLUDE_DIRS})

target_link_libraries(test_planner Eigen3::Eigen robot_params
                      visualization_tools)

install(DIRECTORY robot_resource/meshes robot_resource/urdf
        DESTINATION share/${PROJECT_NAME})

install(
  TARGETS robot_params visualization_tools test_planner test_nearestNeighbors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_package()
