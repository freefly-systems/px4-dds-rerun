from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="px4_ros_offboard",
                executable="px4_ros_offboard",
                name="px4_ros_offboard_node",
                output="screen",
            ),
        ]
    )
