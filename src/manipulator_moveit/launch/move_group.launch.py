import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # -------------------------
    # Paths
    # -------------------------
    moveit_share = get_package_share_directory('manipulator_moveit')

    xacro_path = os.path.join(
        moveit_share,
        'config',
        'manipulator.urdf.xacro',
    )

    srdf_path = os.path.join(moveit_share, 'config', 'manipulator.srdf')
    kinematics_yaml_path = os.path.join(moveit_share, 'config', 'kinematics.yaml')
    ompl_planning_yaml_path = os.path.join(moveit_share, 'config', 'ompl_planning.yaml')

    # ros2_control controllers yaml (너가 방금 만든 controllers.yaml)
    ros2_controllers_yaml_path = os.path.join(moveit_share, 'config', 'ros2_controllers.yaml')

    # MoveIt simple controller manager config (기존 유지)
    moveit_controllers_yaml_path = os.path.join(moveit_share, 'config', 'moveit_controllers.yaml')

    # -------------------------
    # Robot description (URDF from xacro)
    # -------------------------
    robot_description_config = xacro.process_file(xacro_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # -------------------------
    # Robot semantic description (SRDF)
    # -------------------------
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # -------------------------
    # Kinematics (kinematics.yaml)
    # -------------------------
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # -------------------------
    # Planning pipeline (OMPL)
    # -------------------------
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
    with open(ompl_planning_yaml_path, 'r') as f:
        ompl_planning_yaml = yaml.safe_load(f)
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # -------------------------
    # Trajectory execution (MoveIt)
    # -------------------------
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # -------------------------
    # MoveIt controllers (moveit_controllers.yaml)
    # - 이건 MoveIt이 "arm_controller"라는 FollowJointTrajectory를 쓴다고 알고 있게 하는 설정
    # -------------------------
    with open(moveit_controllers_yaml_path, 'r') as f:
        moveit_simple_controllers_yaml = yaml.safe_load(f)

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # -------------------------
    # Planning scene monitor
    # -------------------------
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }

    # -------------------------
    # robot_state_publisher
    # -------------------------
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # -------------------------
    # ros2_control_node (controller_manager)
    # - mock_components/GenericSystem 기반으로 액션 서버 생성
    # -------------------------
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[
            robot_description,
            ros2_controllers_yaml_path,
        ],
    )

    # -------------------------
    # Spawners
    # -------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
        ],
    )

    # -------------------------
    # move_group node
    # -------------------------
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ros2_control_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(arm_controller_spawner)
    ld.add_action(move_group_node)
    return ld
