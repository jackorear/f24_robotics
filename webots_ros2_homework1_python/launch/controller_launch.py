#!/usr/bin/env python

import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Node for the controller
    controller_node = Node(
        package='webots_ros2_homework1_python',
        executable='webots_ros2_homework1_python',
        output='screen',
    )

    return LaunchDescription([
        controller_node
    ])
