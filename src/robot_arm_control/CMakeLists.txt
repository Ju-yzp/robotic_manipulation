# cmake_minimum_required(VERSION 3.16)
# project(ros2_control_ur5e_test LANGUAGES CXX)

# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
#   add_compile_options(-Wall -Wextra)
# endif()

# # set the same behavior for windows as it is on linux
# set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# set(HW_IF_INCLUDE_DEPENDS
#   pluginlib
#   rcpputils
#   hardware_interface
# )

# set(CONTROLLER_INCLUDE_DEPENDS
#   pluginlib
#   rcpputils
#   controller_interface
#   realtime_tools
#   trajectory_msgs
# )

# find_package(backward_ros REQUIRED)
# find_package(ament_cmake REQUIRED)
# find_package(motion_planning REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(trajectory_msgs REQUIRED)

# foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${REF_GEN_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()
# foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

# ## COMPILE
# add_library(
#   ${PROJECT_NAME}
#   SHARED
#   src/robot_arm_hardware.cpp
# )

# target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# target_include_directories(${PROJECT_NAME} PUBLIC
# $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
# )

# ament_target_dependencies(
#   ${PROJECT_NAME} PUBLIC
#   ${HW_IF_INCLUDE_DEPENDS}
#   ${CONTROLLER_INCLUDE_DEPENDS}
# )

# # Export hardware plugins
# pluginlib_export_plugin_description_file(hardware_interface ros2_control_ur5e_test.xml)

# # Export controller plugins
# # pluginlib_export_plugin_description_file(controller_interface ros2_control_demo_example_7.xml)

# add_executable(send_tarjectory src/send_tarjectory.cpp)

# # target_include_directories(send_tarjectory PUBLIC
# # $<BUILD_INTERFACE:${motion_planning_INCLUDE_DIRS}>
# # )

# ament_target_dependencies(
#   send_tarjectory PUBLIC
#   motion_planning
#   rclcpp
#   trajectory_msgs
# )

# install(TARGETS ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME})

# ament_package()

cmake_minimum_required(VERSION 3.16)
project(ros2_control_ur5e_test LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

set(CONTROLLER_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  trajectory_msgs
)

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(motion_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
foreach(Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  ${PROJECT_NAME}
  SHARED
  src/robot_arm_hardware.cpp
  src/robot_arm_ur5e_controller.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

 ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  motion_planning
  ${HW_IF_INCLUDE_DEPENDS}
  ${CONTROLLER_INCLUDE_DEPENDS}
)

# 导出硬件插件
pluginlib_export_plugin_description_file(hardware_interface ros2_control_ur5e_test.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface ros2_control_ur5e_test.xml)

## 编译可执行文件
add_executable(send_trajectory src/send_trajectory.cpp)

ament_target_dependencies(
  send_trajectory
  PUBLIC
  motion_planning
  rclcpp
  trajectory_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS send_trajectory
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
