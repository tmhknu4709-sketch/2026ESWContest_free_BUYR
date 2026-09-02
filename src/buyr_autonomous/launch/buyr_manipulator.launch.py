import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. camera.launch.py 대신 camera_node 단독 노드로 설정
    depthai_camera_node = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak',
        parameters=[
            # 로봇 모델(TF) 오염 방지를 위해 TF 발행 기능 차단
            {'i_publish_tf_from_calibration': False},
            # 컬러 이미지 및 뎁스 정보 활성화
            {'i_enable_color': True},
            {'i_enable_depth': True}
        ],
        output='screen'
    )

    # 2. buyr_detection 패키지의 노드들 설정
    detection_node = Node(
        package='buyr_detection',
        executable='buyr_detection_node',
        name='buyr_detection_node',
        output='screen'
    )

    depth_pub_node = Node(
        package='buyr_detection',
        executable='buyr_depth_pub',
        name='buyr_depth_pub',
        output='screen'
    )

    end_con_node = Node(
        package='buyr_detection',
        executable='buyr_end_con',
        name='buyr_end_con',
        output='screen'
    )

    # LaunchDescription 객체에 모두 담아서 반환
    return LaunchDescription([
        depthai_camera_node,  # 교체된 카메라 단독 노드
        detection_node,
        depth_pub_node,
        end_con_node
    ])
