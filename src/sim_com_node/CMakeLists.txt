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

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS /usr/include /usr/local/include
)

find_library(MUJOCO_LIB mujoco
  HINTS /usr/lib /usr/local/lib
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR "MuJoCo headers or library not found. Set MUJOCO_INCLUDE_DIR and MUJOCO_LIB if installed in a non-standard path.")
endif()

include_directories(${MUJOCO_INCLUDE_DIR})

add_executable(sim_com_node src/sim_com_node.cpp)
target_link_libraries(sim_com_node ${MUJOCO_LIB})
ament_target_dependencies(sim_com_node rclcpp sensor_msgs)

install(TARGETS
  sim_com_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
