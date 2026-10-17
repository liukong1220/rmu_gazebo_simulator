#!/usr/bin/env python3
"""Gazebo 2027 tunnel profile using the shared ATS navigation chain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    simulator_share = FindPackageShare("rmu_gazebo_simulator")
    bringup_share = FindPackageShare("ats_sentry_bringup")

    forwarded = (
        "headless",
        "headless_rendering",
        "use_viewer",
        "use_rviz",
        "planning_grid_owner",
        "launch_planning",
        "use_gazebo_gt_odometry",
        "use_direct_gazebo_lidar_bridge",
        "enable_camera_sensors",
        "solver_mode",
        "enable_test_fault_injection",
    )
    declarations = [DeclareLaunchArgument(name, default_value=value) for name, value in (
        ("headless", "true"),
        ("headless_rendering", "true"),
        ("use_viewer", "false"),
        ("use_rviz", "false"),
        ("planning_grid_owner", "rog_map"),
        ("launch_planning", "true"),
        ("use_gazebo_gt_odometry", "true"),
        ("use_direct_gazebo_lidar_bridge", "true"),
        ("enable_camera_sensors", "false"),
        ("solver_mode", "ilqr"),
        ("enable_test_fault_injection", "false"),
    )]

    arguments = {
        "world": "rmuc_2027",
        "map_yaml": PathJoinSubstitution([bringup_share, "map", "rmuc_2027.yaml"]),
        "prior_pcd_file": PathJoinSubstitution([bringup_share, "pcd", "rmuc_2027.pcd"]),
        "initial_map_to_odom_x": "0.0",
        "initial_map_to_odom_y": "0.0",
        "initial_map_to_odom_z": "0.0",
        "initial_map_to_odom_yaw": "0.0",
        "robot_name": "red_standard_robot1",
        "rviz_config_file": PathJoinSubstitution(
            [simulator_share, "rviz", "ats_gazebo_nav.rviz"]
        ),
    }
    arguments.update({name: LaunchConfiguration(name) for name in forwarded})

    return LaunchDescription(
        declarations
        + [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [simulator_share, "launch", "ats_gazebo_nav.launch.py"]
                    )
                ),
                launch_arguments=arguments.items(),
            )
        ]
    )
