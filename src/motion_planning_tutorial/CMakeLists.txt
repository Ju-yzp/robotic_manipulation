cmake_minimum_required(VERSION 3.10)
project(
  motion_planning_tutorial
  VERSION 0.1.0
  DESCRIPTION "A tutorial for motion planning with UR5e robot"
  LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_FOLDER /usr/local/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_library(robot_description src/robot_description.cpp)
add_library(ikd_Tree src/ikd_Tree.cpp)

target_include_directories(
  ikd_Tree PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

target_link_libraries(ikd_Tree PUBLIC ${PCL_LIBRARIES})

target_include_directories(
  robot_description PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                           ${Eigen3_INCLUDE_DIRS})

target_link_libraries(robot_description PUBLIC Eigen3::Eigen yaml-cpp)

# add_executable(test_ur5e_kinematic test/test_ur5e_kinematic.cpp
# src/ur5e_kinematic.cpp)

# ament_target_dependencies( test_ur5e_kinematic rclcpp sensor_msgs # pcl_ros
# pcl_conversions)

# target_link_libraries(test_ur5e_kinematic robot_description ikd_Tree yaml-cpp
# ${PCL_LIBRARIES})

add_library(controller src/controller.cpp src/non_uniform_bspline.cpp)

target_include_directories(
  controller PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                    ${Eigen3_INCLUDE_DIRS})

add_executable(test_planner test/test_planner.cpp src/ur5e_kinematic.cpp
                            src/collision_detector.cpp src/planner.cpp)

target_include_directories(
  test_planner PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                       ${Eigen3_INCLUDE_DIRS})

target_link_libraries(
  test_planner
  PRIVATE Eigen3::Eigen
          robot_description
          ikd_Tree
          controller
          mujoco::mujoco
          GL
          GLU
          glfw)

install(
  TARGETS robot_description ikd_Tree test_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
# set(TEST_DIR "${PROJECT_SOURCE_DIR}/test")

# file(GLOB cpp_files "${TEST_DIR}/*.cpp")

# foreach(cpp_file ${cpp_files}) get_filename_component(PREFIX ${cpp_file}
# NAME_WE) set(FILE_PREFIX ${PREFIX})

# add_executable(${FILE_PREFIX} ${cpp_file})

# target_include_directories( ${FILE_PREFIX} PRIVATE
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

# target_link_libraries(${FILE_PREFIX} PRIVATE robot_description yaml-cpp)

# install(TARGETS ${FILE_PREFIX} DESTINATION lib/${PROJECT_NAME}) endforeach()
