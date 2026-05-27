from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tts_speaker')
    sound_path = os.path.join(pkg_share, 'sounds')

    return LaunchDescription([
        # 오디오 장치 설정 (PulseAudio, ALSA 등)
        SetEnvironmentVariable(name='SDL_AUDIODRIVER', value='alsa'),

        Node(
            package='tts_speaker',
            executable='tts_speaker_node',
            name='speaker_node',
            output='screen',
            parameters=[
                {'sound_path': sound_path}
            ]
        )
    ])
