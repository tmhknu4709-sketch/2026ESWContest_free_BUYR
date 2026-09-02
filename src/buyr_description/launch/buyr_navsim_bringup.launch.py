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
    map_path = os.path.join(home_dir, 'buyr_map.yaml')
    ekf_config_path = os.path.join(pkg_buyr_description, 'config', 'buyr_ekf_simul.yaml')
    rviz_config_path = os.path.join(home_dir, '.rviz2/buyrnav.rviz')
    
    # [중요] 수정된 nav2_params.yaml 경로
    nav2_params_path = os.path.join(pkg_buyr_description, 'config', 'nav2_params.yaml')

    # 2. 공통 설정
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. URDF 처리
    try:
        robot_description_config = subprocess.check_output(['xacro', urdf_path]).decode('utf-8')
    except Exception as e:
        print(f"ERROR: xacro 처리 중 오류: {e}")
        raise e

    # --- 노드 정의 ---

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_config}]
    )

    spawn_buyr = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'buyr', '-x', '-2.5', '-y', '-2.5', '-z', '0.1'],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 10.0,
            'scan_height': 50,
            'output_frame': 'oak_rgb_camera_optical_frame'
        }],
        remappings=[
            ('image', '/oak/depth/image_raw'),
            ('camera_info', '/oak/depth/camera_info'),
            ('scan', '/depth_scan')
        ]
    )

    # [핵심 수정 부분] Nav2 Bringup 호출 시 인자 최적화
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path,
            'use_composition': 'False',      # 디버깅을 위해 Composition은 잠시 끄는 것이 좋습니다
            'autostart': 'True',             # Lifecycle 노드들을 자동으로 활성화(Active) 시킴
        }.items(),
    )

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
        depth_to_laserscan,
        nav2_bringup,
        rviz2
    ])
