cmake_minimum_required(VERSION 3.10)
project(motion_planner)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find_package(urdfdom REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
# add_subdirectory("third_library/urdf_parser")

##############
## BUILDING ##
##############

add_library(sampler  
src/rrt_connect_sampler.cpp 
src/rrt_sampler.cpp)

target_include_directories(sampler PRIVATE
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
#$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_library/urdf_parser/include>)
target_link_libraries(sampler
  PUBLIC
  Eigen3::Eigen
)

##################
## TEST PROGRAM ##
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
    sampler
    yaml-cpp
    )

    install(TARGETS ${FILE_PREFIX}
            DESTINATION lib/${PROJECT_NAME}) 
endforeach()