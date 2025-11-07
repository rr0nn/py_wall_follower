from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    # ------------------------------------------------------------------
    # Locate packages and default file paths
    # ------------------------------------------------------------------
    pkg_py_wall_follower = Path(FindPackageShare('py_wall_follower').find('py_wall_follower'))
    pkg_tb3_gazebo = Path(FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'))
    pkg_tb3_nav2 = Path(FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2'))

    default_map = 'map.yaml'
    default_params = pkg_py_wall_follower / 'param' / 'nav_param.yaml'

    # ------------------------------------------------------------------
    # Include Gazebo world with TurtleBot3
    # ------------------------------------------------------------------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_tb3_gazebo / 'launch' / 'turtlebot3_maze.launch.py'))
    )

    # ------------------------------------------------------------------
    # Include Nav2 (navigation2 + RViz)
    # ------------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_tb3_nav2 / 'launch' / 'navigation2.launch.py')),
        launch_arguments={
            'use_sim_time': 'True',
            'map': str(default_map),
            'params_file': str(default_params)
        }.items(),
    )

    # ------------------------------------------------------------------
    # Final LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=str(default_map),
            description='Full path to map YAML file'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params),
            description='Full path to Nav2 parameters file'
        ),
        gazebo_launch,
        nav2_launch
    ])