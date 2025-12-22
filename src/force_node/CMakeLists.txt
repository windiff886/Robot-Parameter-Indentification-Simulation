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
)
target_link_libraries(force_controller_component
  Eigen3::Eigen
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
