import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='VideoPath',
            default_value=os.path.join(
                get_package_share_directory('video_test'),
                'video',
                'energe.mp4'
            ),
            description='测试视频路径'
        ),

        Node(
            package='video_test',
            executable='video_test_node',
            name='video_test',
            output='screen',
            emulate_tty=True,
            parameters=[{'VideoPath': LaunchConfiguration('VideoPath')}],
        ),
    ])