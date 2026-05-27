import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_perception_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', 'camera_perception_pkg', 'config'), glob('config/*.yaml')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolov8_node = camera_perception_pkg.yolov8_node:main',
            'yolov8_debug_node = camera_perception_pkg.yolov8_debug_node:main',
            'object_distance_node = camera_perception_pkg.object_distance_node:main',
        ],
    },
)
