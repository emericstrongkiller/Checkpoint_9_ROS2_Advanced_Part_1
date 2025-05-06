import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    my_container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
            ],
            output='screen', 
    )

    attach_server = Node(
        package='my_components',
        executable='manual_composition',
        name='attach_server',
        output='screen',
    )

    return launch.LaunchDescription([my_container, attach_server])