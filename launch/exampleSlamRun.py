import os
import time

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():

    ld = LaunchDescription()

    # Set env var to print messages to stdout immediately
    arg = SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1')
    ld.add_action(arg)

    config = os.path.join(
        get_package_share_directory('low_cost_underwater_slam'),
        'config',
        'slamSettings.yaml'
    )

    ekf_node = Node(
        package='low_cost_underwater_slam',
        executable='ekfNode',
        name='ekfNode',
        output='screen',
        parameters=[config],
        arguments=[]
    )
    ld.add_action(ekf_node)
    slam_node = Node(
        package='low_cost_underwater_slam',
        executable='rosSlamTest',
        name='rosSlamTest',
        output='screen',
        parameters=[config],
        arguments=[])
    ld.add_action(slam_node)

    rviz_node = Node(
    package='rviz2',
    namespace='',
    executable='rviz2',
    name='rviz2',
    arguments=['-d' + os.path.join(get_package_share_directory('low_cost_underwater_slam'), 'config', 'newSLAMView.rviz')])
    ld.add_action(rviz_node)

    return ld
