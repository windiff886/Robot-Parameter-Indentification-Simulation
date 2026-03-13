cmake_minimum_required(VERSION 3.21)
project(robot_parameter_identification_simulation LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenMP)

set(MUJOCO_DIR_ENV $ENV{MUJOCO_DIR})

find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS
    ${MUJOCO_DIR_ENV}/include
    /usr/include
    /usr/local/include
    /opt/mujoco/include
    /opt/mujoco230/include
)

find_library(MUJOCO_LIB mujoco
  HINTS
    ${MUJOCO_DIR_ENV}/lib
    ${MUJOCO_DIR_ENV}/lib64
    /usr/lib
    /usr/local/lib
    /opt/mujoco/lib
    /opt/mujoco/lib64
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIB)
  message(FATAL_ERROR "未找到 MuJoCo。请设置 MUJOCO_DIR，或显式传入 MUJOCO_INCLUDE_DIR / MUJOCO_LIB。")
endif()

set(PROJECT_ROOT_DIR "${CMAKE_SOURCE_DIR}")

add_library(force_controller_core
  src/force_node/src/force_controller.cpp
  src/force_node/src/robot/franka_panda.cpp
  src/force_node/src/robot/robot_kinematics.cpp
  src/force_node/src/robot/robot_dynamics.cpp
  src/force_node/src/robot/regressor.cpp
  src/force_node/src/trajectory/trajectory_generator.cpp
  src/force_node/src/trajectory/fourier_trajectory.cpp
  src/force_node/src/trajectory/trajectory_optimizer.cpp
)
target_include_directories(force_controller_core
  PUBLIC
    src/force_node/include
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(force_controller_core
  PUBLIC
    Eigen3::Eigen
    ${MUJOCO_LIB}
)
target_compile_definitions(force_controller_core
  PUBLIC PROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}"
)

add_library(panda_simulator_core
  src/sim_com_node/src/panda_simulator.cpp
)
target_include_directories(panda_simulator_core
  PUBLIC
    src/sim_com_node/include
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(panda_simulator_core
  PUBLIC
    ${MUJOCO_LIB}
    glfw
)
target_compile_definitions(panda_simulator_core
  PUBLIC PROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}"
)

add_library(identification_core
  src/identification/src/identification.cpp
  src/identification/src/data_loader.cpp
  src/identification/src/algorithms.cpp
  src/identification/src/robot/franka_panda.cpp
  src/identification/src/robot/robot_kinematics.cpp
  src/identification/src/robot/robot_dynamics.cpp
  src/identification/src/robot/regressor.cpp
  src/identification/src/robot/mujoco_panda_dynamics.cpp
  src/identification/src/robot/mujoco_regressor.cpp
  src/identification/src/robot/mujoco_piper_dynamics.cpp
  src/identification/src/robot/mujoco_piper_regressor.cpp
)
target_include_directories(identification_core
  PUBLIC
    src/identification/include
    src/identification/include/robot
    ${MUJOCO_INCLUDE_DIR}
)
target_link_libraries(identification_core
  PUBLIC
    Eigen3::Eigen
    ${MUJOCO_LIB}
)
if(OpenMP_CXX_FOUND)
  target_link_libraries(identification_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(identification_core PUBLIC IDENTIFICATION_USE_OPENMP=1)
endif()
target_compile_definitions(identification_core
  PUBLIC PROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}"
)

add_executable(run_experiment
  src/app/run_experiment.cpp
  src/app/experiment_recorder.cpp
  src/app/simulation_backend.cpp
  src/app/piper_hardware_backend.cpp
)
target_include_directories(run_experiment
  PRIVATE
    src
)
target_link_libraries(run_experiment
  PRIVATE
    force_controller_core
    panda_simulator_core
)

add_executable(identify
  src/identification/src/main.cpp
)
target_link_libraries(identify PRIVATE identification_core)

add_executable(mujoco_identify
  src/identification/src/mujoco_identify.cpp
)
target_link_libraries(mujoco_identify PRIVATE identification_core)

add_executable(dynamics_diagnostic
  src/identification/src/dynamics_diagnostic.cpp
)
target_link_libraries(dynamics_diagnostic PRIVATE identification_core)

add_executable(model_comparison
  src/identification/src/model_comparison.cpp
)
target_link_libraries(model_comparison PRIVATE identification_core)

add_executable(regressor_test
  src/identification/src/regressor_test.cpp
)
target_link_libraries(regressor_test PRIVATE identification_core)
