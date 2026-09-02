from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'buyr_autonomous'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 1. 런치 파일 등록: launch 폴더 안의 모든 .py 파일을 share 폴더로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # 2. 설정 파일 등록: config 폴더 안의 모든 .yaml 파일을 share 폴더로 복사
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taemin',
    maintainer_email='taemin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wheel_control = buyr_autonomous.wheel_control:main',
            'buyr_nav_control = buyr_autonomous.buyr_nav_control:main'
        ],
    },
)
