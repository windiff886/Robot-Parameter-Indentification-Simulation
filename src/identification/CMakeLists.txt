cmake_minimum_required(VERSION 3.8)
project(identification)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 生成 compile_commands.json 供 clangd 使用
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  include/robot
  ${EIGEN3_INCLUDE_DIRS}
)

# Robot Library Sources
file(GLOB ROBOT_SOURCES "src/robot/*.cpp")

# Identification Module Sources
set(ID_SOURCES
  src/identification.cpp
  src/data_loader.cpp
  src/algorithms.cpp
)

add_executable(identify_node
  ${ROBOT_SOURCES}
  ${ID_SOURCES}
  src/main.cpp
)

target_link_libraries(identify_node Eigen3::Eigen)
ament_target_dependencies(identify_node rclcpp)


# Dynamics Diagnostic Tool
add_executable(dynamics_diagnostic
  ${ROBOT_SOURCES}
  src/dynamics_diagnostic.cpp
  src/data_loader.cpp
)
target_link_libraries(dynamics_diagnostic Eigen3::Eigen)
ament_target_dependencies(dynamics_diagnostic rclcpp)

# Model Comparison Tool
add_executable(model_comparison
  ${ROBOT_SOURCES}
  src/model_comparison.cpp
)
target_link_libraries(model_comparison Eigen3::Eigen)

# Regressor Test Tool (验证 MuJoCoRegressor 正确性)
add_executable(regressor_test
  ${ROBOT_SOURCES}
  src/regressor_test.cpp
  src/data_loader.cpp
)
target_link_libraries(regressor_test Eigen3::Eigen)
ament_target_dependencies(regressor_test rclcpp)

# MuJoCo Identify Tool (使用 MuJoCoRegressor 进行参数辨识)
add_executable(mujoco_identify
  ${ROBOT_SOURCES}
  src/mujoco_identify.cpp
  src/data_loader.cpp
  src/algorithms.cpp
)
target_link_libraries(mujoco_identify Eigen3::Eigen)
ament_target_dependencies(mujoco_identify rclcpp)

install(TARGETS identify_node dynamics_diagnostic model_comparison regressor_test mujoco_identify
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
