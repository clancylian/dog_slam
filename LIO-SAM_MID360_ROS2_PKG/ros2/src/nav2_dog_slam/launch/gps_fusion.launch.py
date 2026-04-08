from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, sys


def generate_launch_description():
    try:
        global_config_path = os.path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import DEFAULT_USE_SIM_TIME
    except ImportError:
        print("Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = False

    use_sim_time = DEFAULT_USE_SIM_TIME
    
    nav2_dog_slam_dir = get_package_share_directory('nav2_dog_slam')
    
    gps_ekf_params_file = os.path.join(nav2_dog_slam_dir, 'config', 'gps_ekf.yaml')
    navsat_transform_params_file = os.path.join(nav2_dog_slam_dir, 'config', 'navsat_transform.yaml')
    
    ld = LaunchDescription()
    
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_transform_params_file],
        remappings=[
            ('imu/data', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/gps', 'odometry/gps'),
            ('odometry/filtered', 'odometry/filtered')
        ]
    )
    
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[gps_ekf_params_file],
        remappings=[
            ('odometry/filtered', 'odometry/gps_fused'),
            ('tf', 'tf'),
            ('tf_static', 'tf_static')
        ]
    )
    
    lifecycle_manager_gps = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_gps',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['navsat_transform_node', 'ekf_filter_node']
        }]
    )
    
    ld.add_action(navsat_transform_node)
    ld.add_action(ekf_filter_node)
    ld.add_action(lifecycle_manager_gps)
    
    return ld
