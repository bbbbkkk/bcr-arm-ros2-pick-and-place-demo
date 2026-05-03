#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "bcr_arm",
            package_name="bcr_arm_moveit_config"
        )
        .to_moveit_configs()
    )

    pick_place_demo_node = Node(
        package="bcr_arm_demo",
        executable="pick_place_demo",
        name="pick_place_demo",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        pick_place_demo_node
    ])