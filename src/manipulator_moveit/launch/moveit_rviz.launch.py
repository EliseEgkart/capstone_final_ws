import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, 'r') as file:
        return file.read()


def generate_launch_description():
    moveit_share = get_package_share_directory('manipulator_moveit')

    # =========================================================
    # RViz config
    # =========================================================
    rviz_config = os.path.join(
        moveit_share,
        'config',
        'moveit.rviz'
    )

    # 만약 moveit.rviz가 rviz 폴더에 있다면 위 대신 아래 사용
    # rviz_config = os.path.join(
    #     moveit_share,
    #     'rviz',
    #     'test.rviz'
    # )

    # =========================================================
    # Robot description: URDF from Xacro
    # =========================================================
    xacro_path = os.path.join(
        moveit_share,
        'config',
        'manipulator.urdf.xacro'
    )

    robot_description_config = xacro.process_file(xacro_path)

    robot_description = {
        'robot_description': ParameterValue(
            robot_description_config.toxml(),
            value_type=str
        )
    }

    # =========================================================
    # Robot semantic description: SRDF
    # =========================================================
    robot_description_semantic = {
        'robot_description_semantic': load_file(
            'manipulator_moveit',
            'config/manipulator.srdf'
        )
    }

    # =========================================================
    # Kinematics
    # 중요:
    # kinematics.yaml을 그대로 넣으면 안 되고,
    # 반드시 robot_description_kinematics 아래에 넣어야 함.
    # =========================================================
    robot_description_kinematics = {
        'robot_description_kinematics': load_yaml(
            'manipulator_moveit',
            'config/kinematics.yaml'
        )
    }

    # =========================================================
    # OMPL planning pipeline
    # =========================================================
    ompl_planning_yaml = load_yaml(
        'manipulator_moveit',
        'config/ompl_planning.yaml'
    )

    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }

    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # =========================================================
    # Warehouse config
    # 없어도 RViz marker에는 영향 없음.
    # 일단 유지하되, 경로는 사용자 홈 기준으로 바꾸는 게 좋음.
    # =========================================================
    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': os.path.expanduser('~/warehouse_db.sqlite'),
        'port': 33829,
        'scene_name': '',
        'queries_regex': '.*',
    }

    # =========================================================
    # RViz node
    # =========================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            warehouse_ros_config,
        ]
    )

    return LaunchDescription([
        rviz_node
    ])