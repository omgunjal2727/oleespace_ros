from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Nav2 package share
    nav2_share = get_package_share_directory('nav2_bringup')

    # Arguments
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution(
            [nav2_share, 'params', 'nav2_params.yaml']),
        description='Full path to Nav2 parameter file')

    # Include Nav2 launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_share, 'launch', 'bringup_launch.py'])
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        nav2_launch
    ])
