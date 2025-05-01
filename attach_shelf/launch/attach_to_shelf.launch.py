import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value="0.40", description="Obstacle distance threshold"
    )
    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90", description="Degrees to turn"
    )
    approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="false", description="final approach choice (1||0)"
    )

    # Define pre_approach node
    pre_approach_v2_node = Node(
        package="attach_shelf",
        executable="pre_approach_v2_node",
        name="pre_approach_v2",
        output="screen",
        parameters=[
            {
                "obstacle": LaunchConfiguration("obstacle"),
                "degrees": LaunchConfiguration("degrees"),
                "final_approach": LaunchConfiguration("final_approach"),
            }
        ],
    )

    # Define pre_approach node
    approach_service_node = Node(
        package="attach_shelf",
        executable="approach_service_node",
        name="approach_service_server",
        output="screen"
    )

    # Define rviz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", "/home/user/ros2_ws/src/checkpoint9/attach_shelf/config/cp9_config.rviz"],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        approach_arg,
        approach_service_node,
        pre_approach_v2_node,
        rviz_node,
    ])
