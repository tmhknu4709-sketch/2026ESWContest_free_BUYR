import os
import subprocess
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

    world_path = os.path.join(home_dir, 'buyr_mart') # 확장자(.world) 확인 필요
    urdf_path = os.path.join(pkg_buyr_description, 'urdf', 'buyr.urdf.xacro')
    rviz_config_path = os.path.join(home_dir, '.rviz2/buyrnav.rviz')
    
    robot_description_config = subprocess.check_output(['xacro', urdf_path]).decode('utf-8')

    # 1. Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items(),
    )

    # 2. Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description_config}]
    )

    # 3. Joint State Publisher (바퀴 회전 등 TF 보완)
    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Depth Image to LaserScan (PC에서 변환하여 Orin 부하 감소)
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 10.0,
            'scan_height': 50,
            'output_frame': 'oak_rgb_camera_optical_frame', # URDF와 일치
            'use_sim_time': True
        }],
        remappings=[
            ('image', '/oak/depth/image_raw'),
            ('camera_info', '/oak/depth/camera_info'),
            ('scan', '/depth_scan')
        ]
    )

    # 5. 로봇 스폰
    spawn_buyr = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'buyr', '-x', '-2.5', '-y', '-2.5', '-z', '0.1'],
        output='screen'
    )

    # 6. RViz2
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
        jsp,
        depth_to_laserscan,
        spawn_buyr,
        rviz2
    ])
