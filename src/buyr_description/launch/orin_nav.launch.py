import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    
    try:
        pkg_buyr_description = get_package_share_directory('buyr_description')
    except Exception:
        pkg_buyr_description = os.path.join(home_dir, 'humble_ws/src/buyr_description')

    map_path = os.path.join(home_dir, 'buyr_map.yaml')
    nav2_params_path = os.path.join(pkg_buyr_description, 'config', 'nav2_params.yaml')
    ekf_config_path = os.path.join(pkg_buyr_description, 'config', 'buyr_ekf_simul.yaml')

    # 1. EKF (Odom 데이터 정밀화)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # 2. Nav2 Bringup (AMCL, Planner, Controller 등 실행)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path,
            'use_composition': 'False',
            'autostart': 'True',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ekf_node,
        nav2_bringup
    ])
