from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    # ------------------------------------------------------------------
    # Locate packages and default file paths
    # ------------------------------------------------------------------
    pkg_py_wall_follower = Path(FindPackageShare('py_wall_follower').find('py_wall_follower'))
    pkg_tb3_gazebo = Path(FindPackageShare('turtlebot3_gazebo').find('turtlebot3_gazebo'))
    pkg_tb3_nav2 = Path(FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2'))

    # ------------------------------------------------------------------
    # Default file paths
    # ------------------------------------------------------------------
    default_map = 'map.yaml'
    default_waypoints = 'landmarks.csv'
    default_params = pkg_py_wall_follower / 'config' / 'nav_param.yaml'

    # ------------------------------------------------------------------
    # Declare launch arguments
    # ------------------------------------------------------------------
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=str(default_map),
        description='Full path to map YAML file'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(default_params),
        description='Full path to Nav2 parameters file'
    )

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=str(default_waypoints),
        description='Full path to CSV file containing navigation waypoints'
    )

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
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
        }.items(),
    )

    # ------------------------------------------------------------------
    # Add Waypoint Navigator Node
    # ------------------------------------------------------------------
    waypoint_navigator_node = Node(
        package='py_wall_follower',
        executable='wn',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'csv_path': LaunchConfiguration('waypoints_file'),
            'wait_time': 1.0,
        }]
    )
    
    # Delay the start by 15 seconds to give Nav2 time to fully activate
    delayed_waypoint_node = TimerAction(period=15.0, actions=[waypoint_navigator_node])

    # ------------------------------------------------------------------
    # Final LaunchDescription
    # ------------------------------------------------------------------
    return LaunchDescription([
        map_arg,
        params_arg,
        waypoints_arg,
        gazebo_launch,
        nav2_launch,
        delayed_waypoint_node, # using delayed node to wait for gazebo launch
    ])