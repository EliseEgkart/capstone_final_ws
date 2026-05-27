from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'manipulator_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),


        # =====================================================
        # Config files
        # =====================================================
        # manipulator_manager/config/*.yaml 파일을
        # install/share/manipulator_manager/config 경로에 설치
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='Manipulator manager package for marker-based MoveIt command execution',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'marker_moveit_commander = manipulator_manager.marker_moveit_commander:main',
            'arm_pose_commander = manipulator_manager.arm_pose_commander:main',
            'marker_button_press_commander = manipulator_manager.marker_button_press_commander:main',
            'manipulator_task_manager = manipulator_manager.manipulator_task_manager:main',
        ],
    },
)