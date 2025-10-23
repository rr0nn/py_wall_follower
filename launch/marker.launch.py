from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_wall_follower',
            executable='see_marker',
            name='see_marker'
        ),
        Node(
            package='py_wall_follower',
            executable='point_transformer',
            name='point_transformer',
        )
    ])
