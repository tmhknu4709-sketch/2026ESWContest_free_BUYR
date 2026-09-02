import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 경로 설정
    urdf_file = '/home/taemin/R_buyr.urdf'
    ekf_config = os.path.join(os.path.expanduser('~'), 'humble_ws/src/buyr_autonomous/config/buyr_ekf.yaml')

    return LaunchDescription([
        # 1. 바퀴 제어 노드
        Node(package='buyr_autonomous', executable='wheel_control', name='wheel_control'),

        # 2. IMU 드라이버
        Node(package='iahrs_driver', executable='iahrs_driver', name='iahrs_driver'),

        # 3. 라이다 드라이버 (URG Node)
        Node(package='urg_node', executable='urg_node_driver', name='urg_node',
             parameters=[{'serial_port': '/dev/ttyACM0', 'use_sim_time': False}]),

        # 4. Robot State Publisher (TF)
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             arguments=[urdf_file], parameters=[{'use_sim_time': False}]),

        # 5. EKF (Odom 추정)
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[ekf_config, {'use_sim_time': False}]),

        # 6. SLAM Toolbox (비동기 모드)
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=[{
                 'use_sim_time': False,
                 'odom_frame': 'odom',
                 'base_frame': 'base_footprint',
                 'map_frame': 'map',
                 'mode': 'mapping',
                 'update_rate': 10.0,
                 'minimum_travel_distance': 0.1
             }], output='screen'),
    ])
