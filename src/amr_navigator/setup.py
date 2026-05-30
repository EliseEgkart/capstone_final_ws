import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (os.path.join('share', package_name), ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'launch', 'nav2_bringup'),
            glob('launch/nav2_bringup/*.py'),
        ),
        (
            os.path.join('share', package_name, 'params'),
            glob('params/*.yaml'),
        ),
        (
            os.path.join('share', package_name, 'map'),
            glob('map/*.yaml') + glob('map/*.pgm'),
        ),
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*.rviz'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr2',
    maintainer_email='amr2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x_navigator_node = amr_navigator.x_navigator:main',
            'x_waypoint_node = amr_navigator.x_waypoint_follower_client:main',
            'yaml_waypoint_node = amr_navigator.yaml_waypoint_follower:main',
            'elevator_delivery_manager = amr_navigator.elevator_delivery_manager:main',
            'elevator_delivery_manager2 = amr_navigator.elevator_delivery_manager2:main',
            'elevator_delivery_final = amr_navigator.elevator_delivery_final:main',
            'elevator_delivery_final_with_manipulator = amr_navigator.elevator_delivery_final_with_manipulator:main',
            'elevator_manager_final_with_manipulator = amr_navigator.elevator_manager_final_with_manipulator:main',
            'indoor_students_manager = amr_navigator.indoor_students_manager:main',
        ],
    },
)
