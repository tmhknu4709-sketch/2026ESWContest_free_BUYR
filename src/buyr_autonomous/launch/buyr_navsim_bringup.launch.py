import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_dir = os.path.expanduser('~')
    nav2_params_path = os.path.join(home_dir, 'humble_ws/src/buyr_autonomous/config/nav2_params.yaml')
    ekf_config_path = os.path.join(home_dir, 'humble_ws/src/buyr_autonomous/config/buyr_ekf.yaml')
    urdf_file_path = '/home/taemin/R_buyr.urdf'
    map_yaml_path = '/home/taemin/buyr_bok_map.yaml'

    # URDF 파일 내용 읽기 (robot_state_publisher에 직접 전달하기 위함)
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    nav2_nodes = ['map_server', 'amcl', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']

    return LaunchDescription([
        # [A] 하드웨어 드라이버
        Node(
            package='buyr_autonomous', 
            executable='wheel_control', 
            name='wheel_control', 
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
        Node(
            package='iahrs_driver', 
            executable='iahrs_driver', 
            name='iahrs_driver', 
            parameters=[{'m_bSingle_TF_option': False, 'use_sim_time': False}], 
            output='screen'
        ),
        Node(
            package='urg_node', 
            executable='urg_node_driver', 
            name='urg_node',
            parameters=[{
                'serial_port': '/dev/ttyACM0', 
                'use_sim_time': False,
                'angle_min': -1.5708, # 정면 기준 -90도
                'angle_max': 1.5708   # 정면 기준 +90도
            }],
            output='screen'
        ),

        # [B] 상태 및 위치 추정
        # robot_state_publisher 수정: URDF를 파라미터로 전달하여 타임스탬프 갱신 유도
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_description': robot_desc
            }]
        ),
        
        Node(
            package='robot_localization', 
            executable='ekf_node', 
            name='ekf_filter_node', 
            parameters=[ekf_config_path, {'use_sim_time': False}], 
            output='screen'
        ),

        # [C] Nav2 서버 노드
        Node(
            package='nav2_map_server', 
            executable='map_server', 
            name='map_server', 
            parameters=[nav2_params_path, {'yaml_filename': map_yaml_path, 'use_sim_time': False}]
        ),
        Node(
            package='nav2_amcl', 
            executable='amcl', 
            name='amcl', 
            parameters=[nav2_params_path, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_planner', 
            executable='planner_server', 
            name='planner_server', 
            parameters=[nav2_params_path, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_controller', 
            executable='controller_server', 
            name='controller_server', 
            parameters=[nav2_params_path, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_behaviors', 
            executable='behavior_server', 
            name='behavior_server', 
            parameters=[nav2_params_path, {'use_sim_time': False}]
        ),
        Node(
            package='nav2_bt_navigator', 
            executable='bt_navigator', 
            name='bt_navigator', 
            parameters=[nav2_params_path, {'use_sim_time': False}]
        ),

        # [D] Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager', 
            executable='lifecycle_manager', 
            name='lifecycle_manager_navigation',
            parameters=[{
                'use_sim_time': False,
                'autostart': True, 
                'node_names': nav2_nodes
            }],
            output='screen'
        )
    ])
