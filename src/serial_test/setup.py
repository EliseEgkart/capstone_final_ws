from setuptools import find_packages, setup

package_name = 'serial_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/serial_test.launch.py']),
        ('share/' + package_name, ['launch/nav2_catographer.launch.py']),
        ('share/' + package_name, ['launch/serial_flag_bridge.launch.py']),
        ('share/' + package_name + '/params', ['params/serial_flags.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amr',
    maintainer_email='amr@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = serial_test.test_node_main:main',
            'serial_flag_bridge = serial_test.serial_flag_bridge:main',
            'serial_comm_fsm_node = serial_test.serial_comm_fsm_node:main',
        ],
    },
)
