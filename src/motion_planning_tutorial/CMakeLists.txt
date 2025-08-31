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

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_library(robot_description src/robot_description.cpp)

target_include_directories(
  robot_description PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

target_link_libraries(robot_description PUBLIC Eigen3::Eigen)

set(TEST_DIR "${PROJECT_SOURCE_DIR}/test")

file(GLOB cpp_files "${TEST_DIR}/*.cpp")

foreach(cpp_file ${cpp_files})
  get_filename_component(PREFIX ${cpp_file} NAME_WE)
  set(FILE_PREFIX ${PREFIX})

  add_executable(${FILE_PREFIX} ${cpp_file})

  target_include_directories(
    ${FILE_PREFIX} PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)

  target_link_libraries(${FILE_PREFIX} PRIVATE robot_description yaml-cpp)

  install(TARGETS ${FILE_PREFIX} DESTINATION lib/${PROJECT_NAME})
endforeach()
