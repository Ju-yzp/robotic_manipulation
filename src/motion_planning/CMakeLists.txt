cmake_minimum_required(VERSION 3.12)
project(motion_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(MUJOCO_FOLDER /usr/local/lib/cmake)
find_package(mujoco REQUIRED PATHS ${MUJOCO_FOLDER} NO_DEFAULT_PATH)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

# 构建库
add_library(${PROJECT_NAME}  
  SHARED
  src/inverse_kinematic_solver.cpp 
  src/util.cpp  
  src/scence.cpp 
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  
)

ament_export_targets(export_motion_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_motion_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

##################
## Test Program ##
##################

set(TEST_DIR "${PROJECT_SOURCE_DIR}/test")

file(GLOB CPP_FILES "${TEST_DIR}/*.cpp")

foreach(CPP_FILE ${CPP_FILES})
    get_filename_component(PREFIX ${CPP_FILE} NAME_WE)
    set(FILE_PREFIX ${PREFIX})
    
    add_executable(${FILE_PREFIX} ${CPP_FILE})

    target_include_directories(${FILE_PREFIX} PRIVATE 
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
        
    target_link_libraries(${FILE_PREFIX} PRIVATE 
        ${PROJECT_NAME}
        mujoco::mujoco  
        GL 
        GLU 
        glfw 
    )

    install(TARGETS ${FILE_PREFIX}
            DESTINATION lib/${PROJECT_NAME}) 
endforeach()

ament_package()



