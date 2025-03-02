from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros-rerun',
            executable='px4_ros_rerun.py',
            name='px4_ros_rerun_node',
            output='screen',
        ),
    ]) 