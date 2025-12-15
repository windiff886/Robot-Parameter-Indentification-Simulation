import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory("sim_com_node")
    config_file = os.path.join(pkg_share, "config", "panda_sim_node.yaml")
    model_path = os.path.join(pkg_share, "franka_emika_panda", "scene.xml")

    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="panda_sim_container",
                package="rclcpp_components",
                executable="component_container_mt",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="sim_com_node",
                        plugin="sim_com_node::PandaSimNode",
                        name="panda_sim_node",
                        parameters=[
                            config_file,
                            {"model_path": model_path},
                        ],
                    )
                ],
                output="screen",
            )
        ]
    )
