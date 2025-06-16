cmake_minimum_required(VERSION 3.12)
project(inverse_transform)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_FOLDER /usr/local/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
find_package (Eigen3 3.3 REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp src/util.cpp  src/ur5_inverse_kinematics.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
                          include)

target_link_libraries (${PROJECT_NAME} PUBLIC
                       mujoco::mujoco  
                       GL 
                       GLU 
                       glfw 
                       ${OpenCV_LIBS}
                       Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib)