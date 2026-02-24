#!/usr/bin/env python3
"""
Launch file for fukuro_remotekey.

Usage examples
--------------
# Default (publishes to /cmd_vel):
ros2 launch fukuro_remotekey remotekey_launch.py

# Remap to robot-specific topic:
ros2 launch fukuro_remotekey remotekey_launch.py cmd_vel_topic:=/robot1/cmd_vel

# Override speed defaults:
ros2 launch fukuro_remotekey remotekey_launch.py linear_speed:=1.0 angular_speed:=2.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── Launch arguments ───────────────────────────────────────────────────────
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='cmd_vel',
        description='Target cmd_vel topic for velocity commands',
    )
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Default linear speed (m/s)',
    )
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Default angular speed (rad/s)',
    )
    dribbler_pwm_arg = DeclareLaunchArgument(
        'dribbler_pwm',
        default_value='200',
        description='PWM value sent when dribbler is activated',
    )
    kick_power_arg = DeclareLaunchArgument(
        'kick_power',
        default_value='200',
        description='Power level for kick',
    )
    servo_pos_arg = DeclareLaunchArgument(
        'servo_pos',
        default_value='0.0',
        description='Servo position for kick',
    )

    # ── Node ──────────────────────────────────────────────────────────────────
    remotekey_node = Node(
        package='fukuro_remotekey',
        executable='remotekey',
        name='fukuro_remotekey',
        output='screen',
        emulate_tty=True,       # keep terminal colours / raw input
        prefix='',
        remappings=[
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
        ],
        parameters=[{
            'linear_speed':  LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'dribbler_pwm':  LaunchConfiguration('dribbler_pwm'),
            'kick_power':    LaunchConfiguration('kick_power'),
            'servo_pos':     LaunchConfiguration('servo_pos'),
        }],
    )

    return LaunchDescription([
        cmd_vel_topic_arg,
        linear_speed_arg,
        angular_speed_arg,
        dribbler_pwm_arg,
        kick_power_arg,
        servo_pos_arg,
        remotekey_node,
    ])
