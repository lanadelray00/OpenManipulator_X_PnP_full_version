#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    bringup_pkg = get_package_share_directory('open_manipulator_bringup')
    moveit_pkg = get_package_share_directory('open_manipulator_moveit_config')
    pnp_pkg = get_package_share_directory('openmanipulator_task_executor')

    open_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, 'launch', 'open_manipulator_x.launch.py')
        ),
        launch_arguments={
            'start_rviz': 'false',
            'use_sim': 'false',
        }.items()
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'open_manipulator_x_moveit.launch.py')
        ),
        launch_arguments={
            'start_rviz': 'false',
            'use_sim': 'false',
        }.items()
    )

    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pnp_pkg, 'launch', 'pickandplace_bringup.launch.py')
        )
    )

    return LaunchDescription([
        open_manipulator_launch,
        moveit_launch,
        pick_and_place_launch,
    ])
