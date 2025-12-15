cmake_minimum_required(VERSION 3.8)
project(sim_com_node)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
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
  message(FATAL_ERROR "MuJoCo headers or library not found. Set MUJOCO_DIR, or pass MUJOCO_INCLUDE_DIR and MUJOCO_LIB when configuring.")
endif()

add_library(panda_sim_node_component SHARED src/sim_com_node.cpp)
target_compile_features(panda_sim_node_component PRIVATE cxx_std_17)
target_include_directories(panda_sim_node_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${MUJOCO_INCLUDE_DIR})
target_link_libraries(panda_sim_node_component ${MUJOCO_LIB} glfw)
ament_target_dependencies(panda_sim_node_component rclcpp rclcpp_components sensor_msgs)
ament_target_dependencies(panda_sim_node_component ament_index_cpp)

rclcpp_components_register_nodes(panda_sim_node_component "sim_com_node::PandaSimNode")

install(TARGETS panda_sim_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../config/panda_sim_node.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../franka_emika_panda
  DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp rclcpp_components sensor_msgs ament_index_cpp)

ament_package()
