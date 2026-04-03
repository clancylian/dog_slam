from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
import os, sys


def generate_launch_description():
    try:
        global_config_path = os .path.join(get_package_share_directory('global_config'), '../../src/global_config')
        sys.path.insert(0, global_config_path)
        from global_config import (
            DEFAULT_USE_SIM_TIME,
            NAV2_DEFAULT_MAP_FILE,
            NAV2_DEFAULT_PARAMS_FILE,
            MANUAL_BUILD_MAP,
            AUTO_BUILD_MAP,
            )
    except ImportError:
        # 如果导入失败，使用默认值  
        print(  "Warning: Failed to import global_config, using default values")
        DEFAULT_USE_SIM_TIME = True
        NAV2_DEFAULT_MAP_FILE = "/home/ztl/slam_data/grid_map/map.yaml"
        NAV2_DEFAULT_PARAMS_FILE = "/home/ztl/dog_slam/LIO-SAM_MID360_ROS2_PKG/ros2/src/nav2_dog_slam/config/nav2_params.yaml"
        MANUAL_BUILD_MAP = False
        AUTO_BUILD_MAP = False


    # bringup_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('nav2_dog_slam')
    launch_dir = os.path.join(bringup_dir, 'launch')

    use_sim_time = DEFAULT_USE_SIM_TIME
    params_file = NAV2_DEFAULT_PARAMS_FILE
    map_yaml_file = NAV2_DEFAULT_MAP_FILE
    autostart = True
    log_level = 'info'

    ld = LaunchDescription()


    # slam toolbox node (替代 amcl node)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox_node',
        output='screen',
        parameters=[
            params_file,
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', '/laser_scan'), 
            # ('/Odometry', '/odom/current_pose'),
            ('/odom', '/odom/localization_odom'),  # FAST-LIO 的 odometry 话题
            ('/initialpose', '/initialpose')  # Enable initial pose setting
        ],
        prefix=['taskset -c 5,6'],   # 绑定 CPU 4
        respawn=True,  # 启用自动重启，防止崩溃后系统停止运行
        respawn_delay=2.0  # 重启延迟2秒
    )
    
    
    
    # 网页控制界面节点（ROS2 bridge + Websocket）
    # 这里用 rosbridge + web_video_server 组合
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090},
            {'default_call_service_timeout': 5.0},  # 设置服务调用超时为5.0秒
            {'call_services_in_new_thread': True},  # 在新线程中调用服务
            {'send_action_goals_in_new_thread': True}  # 在新线程中发送动作目标
        ]
    )

    # 使用定时器确保正确的启动顺序
    delayed_slam = TimerAction(period=2.0, actions=[slam_toolbox_node])

    ld.add_action(delayed_slam)
    ld.add_action(rosbridge_websocket)  # 添加 rosbridge_websocket 节点

    return ld