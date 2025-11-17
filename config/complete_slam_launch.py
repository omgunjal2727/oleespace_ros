import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    slam_config_dir = get_package_share_directory('slam_config')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    slam_params_file = os.path.join(slam_config_dir, 'config', 'mapper_params_online_async.yaml')
    ekf_params_file = os.path.join(slam_config_dir, 'config', 'ekf.yaml')
    imu_params_file = os.path.join(slam_config_dir, 'config', 'imu_config.yaml')
    
    # Static transform: base_link -> laser_frame
    static_transform_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # Static transform: base_link -> imu_link
    static_transform_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )
    
    # IMU Serial Node
    imu_serial_node = Node(
        package='imu_serial_driver',
        executable='imu_serial_node',
        name='imu_serial_node',
        parameters=[imu_params_file],
        output='screen'
    )
    
    # Robot Localization (EKF) Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', 'odom')]
    )
    
    # SLAM Toolbox Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        static_transform_base_to_laser,
        static_transform_base_to_imu,
        imu_serial_node,
        ekf_node,
        slam_node,
    ])