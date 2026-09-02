import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 경로 설정
    home_dir = os.path.expanduser('~')
    
    try:
        pkg_buyr_description = get_package_share_directory('buyr_description')
    except Exception:
        pkg_buyr_description = os.path.join(home_dir, 'humble_ws/src/buyr_description')

    world_path = os.path.join(home_dir, 'buyr_mart')
    urdf_path = os.path.join(pkg_buyr_description, 'urdf', 'buyr.urdf.xacro')
    ekf_config_path = os.path.join(pkg_buyr_description, 'config/buyr_ekf_simul.yaml')
    # SLAM용 RViz 설정이 따로 없다면 기본 경로 유지 혹은 새로 지정
    rviz_config_path = os.path.join(home_dir, '.rviz2/buyrnav.rviz')

    # 2. 공통 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. URDF/Xacro 처리
    try:
        robot_description_config = subprocess.check_output(['xacro', urdf_path]).decode('utf-8')
    except Exception as e:
        print(f"ERROR: xacro 처리 중 오류 발생: {e}")
        raise e

    # --- 노드 정의 ---

    # A. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    # B. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True, 
            'robot_description': robot_description_config
        }]
    )

    # C. Gazebo에 로봇 스폰
    spawn_buyr = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'buyr', '-x', '-2.5', '-y', '-2.5', '-z', '0.1'],
        output='screen'
    )

    # D. EKF (로봇 위치 추정)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # E. SLAM Toolbox (Async Mapping Mode)
    # 요청하신 ros2 run 파라미터들을 모두 반영했습니다.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'update_rate': 20.0,
            'transform_publish_period': 0.02,
            'map_update_interval': 1.0,
            'resolution': 0.05,
            'max_laser_range': 5.0,
            'minimum_travel_distance': 0.02,
            'minimum_travel_heading': 0.02,
            'do_loop_closure': True,
            'mode': 'mapping',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'scan_matching_minimum_score': 0.1,
            'stack_size_to_use': 40000000, # 대규모 맵 대비 스택 사이즈 (선택사항)
            'CeresSolver.options.num_threads': 8
        }]
    )

    # F. RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        rsp,
        spawn_buyr,
        ekf_node,
        slam_toolbox,
        rviz2
    ])
