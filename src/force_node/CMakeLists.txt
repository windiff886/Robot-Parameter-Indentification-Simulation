cmake_minimum_required(VERSION 3.8)
project(force_node)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glfw3 3.3 REQUIRED)

set(MUJOCO_DIR_ENV $ENV{MUJOCO_DIR})

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS /usr/include /usr/local/include
  PATHS
    ${MUJOCO_DIR_ENV}/include
    /opt/mujoco/include
    /opt/mujoco230/include
    /opt/homebrew/opt/mujoco/include
)

find_library(MUJOCO_LIB mujoco
  HINTS /usr/lib /usr/local/lib
  PATHS
    ${MUJOCO_DIR_ENV}/lib
    ${MUJOCO_DIR_ENV}/lib64
    /opt/mujoco/lib
    /opt/mujoco/lib64
    /opt/homebrew/opt/mujoco/lib
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR "MuJoCo headers or library not found.")
endif()

# 力矩控制器组件库
add_library(force_controller_component SHARED 
  src/force_controller_node.cpp
  src/robot/franka_panda.cpp
  src/robot/robot_kinematics.cpp
  src/robot/robot_dynamics.cpp
  src/robot/regressor.cpp
  src/trajectory/trajectory_generator.cpp
  src/trajectory/fourier_trajectory.cpp
  src/trajectory/trajectory_optimizer.cpp
)
target_compile_features(force_controller_component PRIVATE cxx_std_17)
target_include_directories(force_controller_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(force_controller_component
  Eigen3::Eigen
  ${MUJOCO_LIB}
  glfw
)
ament_target_dependencies(force_controller_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs

  ament_index_cpp
)

rclcpp_components_register_nodes(force_controller_component "force_node::ForceControllerNode")

# 安装
install(TARGETS force_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/force_controller_node.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs std_msgs ament_index_cpp)



ament_package()

