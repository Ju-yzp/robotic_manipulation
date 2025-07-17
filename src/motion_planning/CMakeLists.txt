cmake_minimum_required(VERSION 3.12)
project(motion_planning)

#################
###  Setting  ###
#################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MUJOCO_FOLDER /usr/local/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
find_package (Eigen3 3.3 REQUIRED)

##################
## Build Library##
##################

add_library(${PROJECT_NAME}  
            src/inverse_kinematic_solver.cpp 
            src/util.cpp  
            src/scence.cpp)

include_directories(include )

target_link_libraries (${PROJECT_NAME} PUBLIC
                       Eigen3::Eigen)


##################
## Test Program ##
##################

set(TEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test")

file(GLOB CPP_FILES "${TEST_DIR}/*.cpp")

foreach(CPP_FILE ${CPP_FILES})

    get_filename_component(PREFIX ${CPP_FILE} NAME_WE)
 
    set(FILE_PREFIX ${PREFIX})
    
    add_executable(${FILE_PREFIX} ${CPP_FILE})

    target_link_libraries(${FILE_PREFIX} PRIVATE 
                          ${PROJECT_NAME}
                          mujoco::mujoco  
                          GL 
                          GLU 
                          glfw )

    install(TARGETS ${FILE_PERFIX}
            DESTINATION lib)
endforeach()

