from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    config_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'src',
        'slam_config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
        ),
    ])
