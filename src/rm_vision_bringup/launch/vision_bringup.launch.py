import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

import yaml

def generate_launch_description():
    camera_type = LaunchConfiguration('camera_type')
    use_serial = LaunchConfiguration('use_serial')

    declare_camera_type_cmd = DeclareLaunchArgument(
        'camera_type',
        default_value='dahua',
        description='Available camera types: dahua, daheng')
    declare_use_serial_cmd = DeclareLaunchArgument(
        'use_serial',
        default_value='False',
        description='Whether use serial port')

    params_file = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'params.yaml')

    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_description'), 'urdf', 'gimbal.urdf.xacro')])

    with open(params_file, 'r') as f:
        dahua_camera_params = yaml.safe_load(f)['/dahua_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        daheng_camera_params = yaml.safe_load(f)['/daheng_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']

    detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::RgbDetectorNode',
        name='armor_detector',
        parameters=[detector_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    dahua_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(PythonExpression(["'", camera_type, "'=='dahua'"])),
        composable_node_descriptions=[
            detector_node,
            ComposableNode(
                package='dahua_camera',
                plugin='dahua_camera::DahuaCameraNode',
                name='dahua_camera_node',
                parameters=[dahua_camera_params, {'use_sensor_data_qos': True}],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    daheng_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        condition=IfCondition(PythonExpression(["'", camera_type, "'=='daheng'"])),
        composable_node_descriptions=[
            detector_node,
            ComposableNode(
                package='daheng_camera',
                plugin='daheng_camera::DahengCameraNode',
                name='daheng_camera_node',
                parameters=[daheng_camera_params, {'use_serial_data_qos': True}],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'publish_frequency': 1000.0}]
    )

    ist_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ist_serial_driver'), 'launch', 'serial_driver.launch.py')),
        condition=IfCondition(use_serial)
    )

    return LaunchDescription([
        dahua_container,
        daheng_container,
        processor_node,
        robot_state_publisher,
        ist_serial_launch,
    ])
