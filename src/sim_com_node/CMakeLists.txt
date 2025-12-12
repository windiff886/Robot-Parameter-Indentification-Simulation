cmake_minimum_required(VERSION 3.8)
project(sim_com_node)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

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

include_directories(${MUJOCO_INCLUDE_DIR})

add_executable(sim_com_node src/sim_com_node.cpp)
target_link_libraries(sim_com_node ${MUJOCO_LIB})
ament_target_dependencies(sim_com_node rclcpp sensor_msgs)

install(TARGETS
  sim_com_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
