#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pca_params = os.path.join(
        get_package_share_directory('ros2_pca9685'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([

        # ============================
        # PCA9685 Servo Driver
        # ============================
        Node(
            package='ros2_pca9685',
            executable='ros2_pca9685_node',
            name='pca9685_node',
            output='screen',
            parameters=[pca_params]
        ),

        # ============================
        # Motion Node
        # Handles /cmd_vel → GaitController internally
        # (do NOT also launch gait_controller separately)
        # ============================
        Node(
            package='sharky_control',
            executable='motion_node',
            name='motion_node',
            output='screen'
        ),

    ])
