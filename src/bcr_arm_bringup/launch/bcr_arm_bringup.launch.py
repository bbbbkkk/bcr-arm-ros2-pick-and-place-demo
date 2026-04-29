#!/usr/bin/env python3

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 获取各个包的 share 路径
    gazebo_share = get_package_share_directory('bcr_arm_gazebo')
    moveit_share = get_package_share_directory('bcr_arm_moveit_config')

    # 被 include 的 launch 文件路径
    gazebo_launch = Path(gazebo_share) / 'launch' / 'bcr_arm_gazebo.launch.py'
    move_group_launch = Path(moveit_share) / 'launch' / 'move_group.launch.py'
    rviz_launch = Path(moveit_share) / 'launch' / 'moveit_rviz.launch.py'

    # 1. 启动 Gazebo + 机械臂 + ros2_control
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo_launch))
    )

    # 2. 延迟启动 move_group
    # 给 Gazebo 和 controller 一点启动时间
    start_move_group = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(move_group_launch))
            )
        ]
    )

    # 3. 延迟启动 RViz
    # 等 move_group 基本起来之后再打开 RViz
    start_rviz = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rviz_launch))
            )
        ]
    )

    # 4. 延迟添加 MoveIt PlanningScene 障碍物
    # 这个脚本只负责让 MoveIt / RViz 知道障碍物
    add_collision_objects = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='bcr_arm_moveit_config',
                executable='add_collision_objects.py',
                name='add_collision_objects',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        start_gazebo,
        start_move_group,
        start_rviz,
        add_collision_objects,
    ])