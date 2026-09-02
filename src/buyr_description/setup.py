from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'buyr_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # --- 아래 항목들을 추가하세요 ---
        # launch 폴더 안의 모든 .py 파일 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # config 폴더 안의 모든 파일 설치 (yaml 등)
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        # urdf 폴더가 있다면 추가 (없어도 경로는 맞춰두는 게 좋습니다)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # rviz 폴더가 있다면 추가
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taemin',
    maintainer_email='taemin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 실행 노드가 있다면 여기에 작성
        ],
    },
)
