cmake_minimum_required(VERSION 3.5)
project(visual_path_planning)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17) 
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "visualization_msgs"
  "geometry_msgs"
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )


# install(DIRECTORY
#   rviz
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
