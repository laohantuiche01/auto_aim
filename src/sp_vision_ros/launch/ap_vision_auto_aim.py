#!/usr/bin/env python3

"""
SP Vision ROS Launch File

支持两种模式:
1. 真实机器人模式 (use_simulator:=false): 启动 camera_node + gimbal_node + auto_aim_node + armor_solver_node
2. 模拟器模式 (use_simulator:=true): 只启动 auto_aim_node + armor_solver_node，使用 Godot 模拟器的 topics

用法:
    # 启动系统（使用默认参数文件）
    ros2 launch sp_vision_ros sp_vision_bringup.launch.py

    # Godot 模拟器模式
    ros2 launch sp_vision_ros sp_vision_bringup.launch.py use_simulator:=true

参数说明:
    use_simulator: 是否使用模拟器模式 (true/false)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('sp_vision_ros')
    params_dir = os.path.join(pkg_share, 'config', 'params')

    # 声明 launch 参数
    use_simulator_arg = DeclareLaunchArgument(
        'use_simulator',
        default_value='true',
        description='是否使用模拟器模式 (true/false)'
    )

    use_simulator = LaunchConfiguration('use_simulator')

    # Auto Aim Node - 始终启动
    auto_aim_node = Node(
        package='sp_vision_ros',
        executable='auto_aim_node',
        name='auto_aim',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(params_dir, 'auto_aim_params.yaml')]
    )


    delayed_auto_aim_node = TimerAction(
        period=0.5,
        actions=[auto_aim_node]
    )

    # ===== Launch Description =====
    return LaunchDescription([
        use_simulator_arg,
        delayed_auto_aim_node
    ])
