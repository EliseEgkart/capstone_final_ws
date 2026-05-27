# file: amr_navigator/launch/yaml_waypoint_follower.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_navigator')
    default_yaml = os.path.join(pkg_share, 'params', '1F_IT_building.yaml')

    waypoints_file = LaunchConfiguration('waypoints_file')
    sequence = LaunchConfiguration('sequence')
    frame_id = LaunchConfiguration('frame_id')
    auto_start = LaunchConfiguration('auto_start')
    repeat = LaunchConfiguration('repeat')
    repeat_delay_sec = LaunchConfiguration('repeat_delay_sec')

    # Nav <-> Serial FSM (internal topics)
    wp_arrive_topic = LaunchConfiguration('wp_arrive_topic')  # yaml_waypoint_node publishes ARRIVED:<wp>
    wp_done_topic = LaunchConfiguration('wp_done_topic')      # yaml_waypoint_node waits DONE:<wp>

    arrive_prefix = LaunchConfiguration('arrive_prefix')
    done_prefix = LaunchConfiguration('done_prefix')
    wait_for_done = LaunchConfiguration('wait_for_done')
    require_done_match = LaunchConfiguration('require_done_match')
    continue_on_miss = LaunchConfiguration('continue_on_miss')

    # Serial port (Robot A side)
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    timeout = LaunchConfiguration('timeout')
    line_ending = LaunchConfiguration('line_ending')

    # comm.md mapping & task plan (Robot A scenario)
    ws_map_yaml = LaunchConfiguration('ws_map_yaml')
    task_plan_yaml = LaunchConfiguration('task_plan_yaml')
    task_policy = LaunchConfiguration('task_policy')
    pick_task = LaunchConfiguration('pick_task')
    place_task = LaunchConfiguration('place_task')

    # Reliability
    done_timeout_sec = LaunchConfiguration('done_timeout_sec')
    resend_sec = LaunchConfiguration('resend_sec')
    max_resends = LaunchConfiguration('max_resends')
    fail_policy = LaunchConfiguration('fail_policy')  # stop | skip_waypoint

    return LaunchDescription([
        DeclareLaunchArgument('waypoints_file', default_value=default_yaml, description='Path to waypoints YAML file'),
        DeclareLaunchArgument('sequence', default_value='', description='Override waypoint sequence (e.g. [a,b])'),
        DeclareLaunchArgument('frame_id', default_value='map', description='Frame ID for waypoints'),
        DeclareLaunchArgument('auto_start', default_value='true', description='Start sequence immediately'),
        DeclareLaunchArgument('repeat', default_value='true', description='Repeat sequence after completion'),
        DeclareLaunchArgument('repeat_delay_sec', default_value='0.0', description='Delay before repeating (seconds)'),
        DeclareLaunchArgument('continue_on_miss', default_value='false', description='Continue if waypoint missed'),

        # Internal handshake topics with yaml_waypoint_node
        DeclareLaunchArgument('wp_arrive_topic', default_value='wp_arrived', description='ARRIVED:<wp> from nav node'),
        DeclareLaunchArgument('wp_done_topic', default_value='wp_done', description='DONE:<wp> to nav node'),

        # yaml_waypoint_node prefixes (colon style)
        DeclareLaunchArgument('arrive_prefix', default_value='ARRIVED', description='Nav arrival prefix (ARRIVED:<wp>)'),
        DeclareLaunchArgument('done_prefix', default_value='DONE', description='Nav done prefix (DONE:<wp>)'),
        DeclareLaunchArgument('wait_for_done', default_value='true', description='Nav waits DONE:<wp> before next'),
        DeclareLaunchArgument('require_done_match', default_value='true', description='Nav requires done include wp name'),

        # Serial port args (Robot A)
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port'),
        DeclareLaunchArgument('baudrate', default_value='115200', description='Serial baudrate'),
        DeclareLaunchArgument('timeout', default_value='0.1', description='Serial timeout seconds'),
        DeclareLaunchArgument('line_ending', default_value='\n', description='Line ending for serial TX'),

        # Mapping: waypoint_name -> WSx (comm.md)
        DeclareLaunchArgument(
            'ws_map_yaml',
            default_value='{work_station1: WS1, work_station2: WS2, work_station3: WS3}',
            description='YAML dict mapping waypoint name to WS id',
        ),
        # Task plan: WSx -> [TASK...]
        DeclareLaunchArgument(
            'task_plan_yaml',
            default_value='{WS1: [PICK3], WS2: [PLACE3, PICK3], WS3: [PLACE3, PICK3]}',
            description='YAML dict mapping WS to tasks list (scenario)',
        ),
        DeclareLaunchArgument(
            'task_policy',
            default_value='first_pick_then_place_pick',
            description='FSM task policy: ws_plan | first_pick_then_place_pick',
        ),
        DeclareLaunchArgument('pick_task', default_value='PICK3', description='FSM pick task name'),
        DeclareLaunchArgument('place_task', default_value='PLACE3', description='FSM place task name'),

        # Timeouts / resend
        DeclareLaunchArgument('done_timeout_sec', default_value='120.0', description='Timeout waiting DONE per task'),
        DeclareLaunchArgument('resend_sec', default_value='2.0', description='Resend ARRIVED interval seconds'),
        DeclareLaunchArgument('max_resends', default_value='5', description='Max resends per task'),
        DeclareLaunchArgument('fail_policy', default_value='stop', description='On FAIL: stop | skip_waypoint'),

        # 1) Nav node (unchanged)
        Node(
            package='amr_navigator',
            executable='yaml_waypoint_node',
            name='yaml_waypoint_follower',
            output='screen',
            parameters=[
                {'waypoints_file': waypoints_file},
                {'sequence': ParameterValue(sequence, value_type=str)},
                {'frame_id': frame_id},
                {'auto_start': auto_start},
                {'repeat': repeat},
                {'repeat_delay_sec': repeat_delay_sec},
                {'arrive_topic': wp_arrive_topic},
                {'done_topic': wp_done_topic},
                {'arrive_prefix': arrive_prefix},
                {'done_prefix': done_prefix},
                {'wait_for_done': wait_for_done},
                {'require_done_match': require_done_match},
                {'continue_on_miss': continue_on_miss},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        # 2) Serial FSM (Robot A side)
        Node(
            package='serial_test',
            executable='serial_comm_fsm_node',
            name='serial_comm_fsm_node',
            output='screen',
            parameters=[
                {'role': 'autonomy'},

                # Nav handshake topics
                {'wp_arrive_topic': wp_arrive_topic},
                {'wp_done_topic': wp_done_topic},
                {'arrive_prefix': arrive_prefix},
                {'done_prefix': done_prefix},

                # Serial port
                {'port': port},
                {'baudrate': baudrate},
                {'timeout': timeout},
                {'line_ending': ParameterValue(line_ending, value_type=str)},

                # Scenario plan
                {'ws_map_yaml': ParameterValue(ws_map_yaml, value_type=str)},
                {'task_plan_yaml': ParameterValue(task_plan_yaml, value_type=str)},
                {'task_policy': task_policy},
                {'pick_task': pick_task},
                {'place_task': place_task},

                # Reliability
                {'done_timeout_sec': done_timeout_sec},
                {'resend_sec': resend_sec},
                {'max_resends': max_resends},
                {'fail_policy': fail_policy},
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
