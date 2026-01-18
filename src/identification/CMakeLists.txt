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


install(TARGETS identify_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
