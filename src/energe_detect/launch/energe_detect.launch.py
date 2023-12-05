import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
        config = os.path.join(
            get_package_share_directory('energe_detect'), 'config', 'energe_detect.yaml')

        detect_node = Node(
            package='energe_detect',
            executable='energe_detect_node',
            name='energe_detect_test',
            output='screen',
            emulate_tty=True,
            parameters=[config])

        return LaunchDescription([detect_node])