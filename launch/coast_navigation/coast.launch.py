# Author: Megan Neville
# Description: Launch left or right configuration using argument

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    launch_dir = os.path.join(pkg_share, 'launch')

    # -----------------------------
    # Declare direction argument
    # -----------------------------
    declare_direction_cmd = DeclareLaunchArgument(
        'direction',
        default_value='left',
        description='Choose direction: left or right'
    )

    direction = LaunchConfiguration('direction')

    launch_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'coast_navigation/leftcoast.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", direction, "' == 'left'"])
        )
    )

    launch_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'coast_navigation/rightcoast.launch.py')
        ),
        condition=IfCondition(
            PythonExpression(["'", direction, "' == 'right'"])
        )
    )

    ld = LaunchDescription()

    ld.add_action(declare_direction_cmd)
    ld.add_action(launch_left)
    ld.add_action(launch_right)

    return ld