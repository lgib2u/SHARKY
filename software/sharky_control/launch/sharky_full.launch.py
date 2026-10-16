#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # ============================
        # PCA9685 Servo Driver
        # ============================
        Node(
            package='ros2_pca9685',
            executable='ros2_pca9685_node',
            name='pca9685',
            output='screen'
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

        # ============================
        # Web Interface + Camera
        # Flask on :5000, camera via cv2.VideoCapture(0)
        # (do NOT also run v4l2_camera_node — device conflict)
        # ============================
        Node(
            package='sharky_control',
            executable='web_interface',
            name='web_interface',
            output='screen'
        ),

    ])
