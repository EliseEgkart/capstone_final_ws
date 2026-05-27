from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tts_speaker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'sounds'), glob('sounds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moonshot',
    maintainer_email='ky942400@gmail.com',
    description='ROS2 TTS speaker node for voice and sound playback',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'tts_speaker_node = tts_speaker.tts_speaker_node:main',
        ],
    },
)