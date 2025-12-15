from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """启动力矩控制器节点（参数从配置文件直接读取）."""
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="force_controller_container",
                package="rclcpp_components",
                executable="component_container",
                namespace="",
                composable_node_descriptions=[
                    ComposableNode(
                        package="force_node",
                        plugin="force_node::ForceControllerNode",
                        name="force_controller_node",
                    )
                ],
                output="screen",
            )
        ]
    )
