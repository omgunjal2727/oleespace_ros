import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    slam_config_dir = get_package_share_directory('slam_config')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # --- CONFIG FILE PATHS ---
    slam_params_file = os.path.join(slam_config_dir, 'config', 'mapper_params_online_async.yaml')
    ydlidar_params_file = os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml')
    imu_params_file = os.path.join(slam_config_dir, 'config', 'imu_config.yaml')
    # Assuming the EKF config file is also in the config directory:
    ekf_params_file = os.path.join(slam_config_dir, 'config', 'ekf.yaml')

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

    # 3. ODOMETRY (RF2O and EKF Filter)
    # RF2O generates /odom_rf2o topic, but DOES NOT publish TF
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,            # CRITICAL: EKF will publish TF
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 50.0,
            'max_range': 8.0,
            'min_range': 0.25,
            'min_vel': 0.0,
            'max_icp_iter': 30,
            'smooth_odom': True,
        }],
    )

    # EKF consumes /odom_rf2o, smoothes it, and publishes the final odom -> base_link TF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_filter_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        # EKF output is remapped to /odom for SLAM Toolbox consumption
        remappings=[('odometry/filtered', '/odom')] 
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
        # TF Publishers
        static_transform_base_to_laser,
        static_transform_base_to_imu,
        
        # Drivers
        ydlidar_node,
        imu_serial_node,
        
        # Odometry Pipeline (RF2O -> EKF)
        rf2o_node,
        ekf_node, 
        
        # SLAM
        slam_node,
    ])