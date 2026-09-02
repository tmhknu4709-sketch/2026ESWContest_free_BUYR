from setuptools import setup

package_name = 'buyr_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'buyr_detection_node = buyr_detection.buyr_detection_node:main',
        'target_class_publisher = buyr_detection.target_class_publisher:main',
        'keyboard_class_switcher = buyr_detection.keyboard_class_switcher:main',
        'buyr_end_con_hrz_B = buyr_detection.buyr_end_con_hrz_B:main',
        'buyr_end_con_A = buyr_detection.buyr_end_con_A:main' ,
        'buyr_end_con_hrz_A = buyr_detection.buyr_end_con_hrz_A:main' ,
        'buyr_end_con_hrz_C = buyr_detection.buyr_end_con_hrz_C:main' ,
        'test = buyr_detection.test:main' ,
        'buyr_depth_pub = buyr_detection.buyr_depth_pub:main',
        'buyr_master_con = buyr_detection.buyr_master_con:main',
        'buyr_suc_con = buyr_detection.buyr_suc_con:main',
        'buyr_end_con_B = buyr_detection.buyr_end_con_B:main'
        ],
    },
)
