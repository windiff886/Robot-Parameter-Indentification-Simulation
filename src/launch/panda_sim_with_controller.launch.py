"""
一次性启动 MuJoCo 仿真节点和力矩控制器节点。

使用方法:
    ros2 launch src/launch/panda_sim_with_controller.launch.py
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """启动仿真节点和力矩控制器节点."""
    # 获取包路径
    sim_com_pkg = get_package_share_directory("sim_com_node")

    # 仿真节点配置
    sim_config_file = os.path.join(sim_com_pkg, "config", "panda_sim_node.yaml")
    model_path = os.path.join(sim_com_pkg, "franka_emika_panda", "scene.xml")

    return LaunchDescription(
        [
            # 单个容器中运行两个节点，提高通信效率
            ComposableNodeContainer(
                name="panda_sim_container",
                package="rclcpp_components",
                executable="component_container_mt",  # 多线程容器
                namespace="",
                composable_node_descriptions=[
                    # MuJoCo 仿真节点
                    ComposableNode(
                        package="sim_com_node",
                        plugin="sim_com_node::PandaSimNode",
                        name="panda_sim_node",
                        parameters=[
                            sim_config_file,
                            {"model_path": model_path},
                        ],
                    ),
                ],
                output="screen",
            ),
            # 力矩控制器节点 (独立进程运行，避免 MuJoCo 冲突)
            # 使用 Node 而不是 ComposableNode
            # 注意：force_node 是基于 rclcpp::Node 的组件，这里作为独立节点运行
            # 需要确认 force_node 是否有独立的可执行文件 target
            # 通常组件包会有一个 standalone 的 executable (e.g. force_controller_node_exe?)
            # 如果没有，我们需要使用 ros2 run rclcpp_components component_container 并加载它？
            # 或者，修改 CMakeLists.txt 生成一个可执行文件。
            # 这里假设它只是一个组件库。
            # 如果它只是组件，我们必须用容器。
            # 为它单独启动一个容器！
            ComposableNodeContainer(
                name="force_control_container",
                package="rclcpp_components",
                executable="component_container", # 使用普通容器
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="force_node",
                        plugin="force_node::ForceControllerNode",
                        name="force_controller_node",
                    ),
                ],
                output="screen",
            )
        ]
    )
