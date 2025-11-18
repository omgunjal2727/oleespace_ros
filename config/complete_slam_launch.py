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
    ydlidar_params_file = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')
    imu_params_file = os.path.join(slam_config_dir, 'config', 'imu_config.yaml')

    # 1. STATIC TRANSFORMS (Physical Mounting)
    # base_link -> laser_frame (0.15m up)
    static_transform_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.15', '0', '0', '0', 'base_link', 'laser_frame'],
        output='screen'
    )
    
    # base_link -> imu_link (0.05m up)
    static_transform_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )

    # 2. SENSOR DRIVERS
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[ydlidar_params_file]
    )
    
    imu_serial_node = Node(
        package='imu_serial_driver',
        executable='imu_serial_node',
        name='imu_serial_node',
        parameters=[imu_params_file],
        output='screen'
    )

    # 3. ODOMETRY (The "Virtual Wheels")
    # This calculates odom -> base_link from the Lidar
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,     # CRITICAL: This publishes odom -> base_link
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 4. SLAM TOOLBOX
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        static_transform_base_to_laser,
        static_transform_base_to_imu,
        ydlidar_node,
        imu_serial_node,
        rf2o_node,
        slam_node,
    ])