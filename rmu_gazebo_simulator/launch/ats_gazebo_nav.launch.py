# Copyright 2026 ATS 2026 Sentry Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Gazebo world + ATS navigation chain.

Chain wired by this file:

    Gazebo (SwerveDrive4WS chassis, mid360 gpu_lidar + imu, /clock)
      -> ros_gz_bridge  PointCloud2 / Imu
      -> gz_livox_bridge            (format adapter, livox CustomMsg)
      -> point_lio                  cloud_registered + aft_mapped_to_init
      -> loam_interface             /registered_scan + /lidar_odometry
      -> sensor_scan_generation     /odometry + odom->gimbal_yaw_odom TF
      -> localization_fusion        /localization + /localization/status
      -> ats_rog_map                /rog_map/*
      -> ats_rog_map_adapter        /rc_esdf/planning_grid
      -> minco_planner              /minco/raw_path + /minco/reference_path
      -> ats_swerve_mpc             /cmd_vel_mpc
      -> gz_chassis_cmd_adapter     /motion_control + <robot>/cmd_vel
      -> Gazebo chassis

Ownership rules enforced here:

* ``/cmd_vel_mpc``            single publisher: ats_swerve_mpc.
* ``/motion_control``         single publisher: gz_chassis_cmd_adapter.
* ``/rc_esdf/planning_grid``  single publisher: ats_rog_map_adapter.
* ``odom -> base_footprint`` / ``odom -> gimbal_yaw_odom``  single publisher:
  sensor_scan_generation.
* ``gimbal_yaw_odom -> front_mid360``  single publisher: the static TF below.
  The real-vehicle bringup publishes it too, but only when
  ``use_sim_time:=false``, so the two never coexist.

``fake_vel_transform`` and ``chassis_vel_transform`` are intentionally NOT
started: gz_chassis_cmd_adapter already performs the big-yaw rotation and is
the single owner of the chassis command in this profile.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map_yaml")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")
    planning_grid_owner = LaunchConfiguration("planning_grid_owner")
    point_lio_scan_line = LaunchConfiguration("point_lio_scan_line")
    livox_update_rate_hz = LaunchConfiguration("livox_update_rate_hz")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_test_fault_injection = LaunchConfiguration("enable_test_fault_injection")
    require_gimbal_status = LaunchConfiguration("require_gimbal_status")
    projection_rate_hz = LaunchConfiguration("projection_rate_hz")
    initial_map_to_odom_x = LaunchConfiguration("initial_map_to_odom_x")
    initial_map_to_odom_y = LaunchConfiguration("initial_map_to_odom_y")
    initial_map_to_odom_z = LaunchConfiguration("initial_map_to_odom_z")
    initial_map_to_odom_roll = LaunchConfiguration("initial_map_to_odom_roll")
    initial_map_to_odom_pitch = LaunchConfiguration("initial_map_to_odom_pitch")
    initial_map_to_odom_yaw = LaunchConfiguration("initial_map_to_odom_yaw")

    rog_map_owned = IfCondition(
        PythonExpression(["'", planning_grid_owner, "' == 'rog_map'"])
    )

    declarations = [
        DeclareLaunchArgument(
            "world",
            default_value="rmuc_2025",
            description="Gazebo world name; resolves resource/worlds/<world>_world.sdf",
        ),
        DeclareLaunchArgument(
            "world_sdf_path",
            default_value="",
            description=(
                "Explicit Gazebo world SDF path. Empty keeps the world-name "
                "resolution above."
            ),
        ),
        DeclareLaunchArgument(
            "map_yaml",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ats_sentry_bringup"), "map", "rmuc_2025.yaml"]
            ),
            description=(
                "Static planning map YAML. The PGM is resolved from its 'image' "
                "field, so the map is never copied into the simulator package."
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ats_sentry_bringup"), "params", "node_params.yaml"]
            ),
            description="Root-owned navigation parameter YAML",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("robot_name", default_value="red_standard_robot1"),
        DeclareLaunchArgument(
            "initial_map_to_odom_x",
            default_value="-0.18",
            description=(
                "Gazebo-only initial map->odom x [m]. For rmuc_2025 this is "
                "the robot spawn x plus the PGM origin x; it registers the "
                "static map to Point-LIO's local odom without ground-truth input."
            ),
        ),
        DeclareLaunchArgument(
            "initial_map_to_odom_y",
            default_value="0.06",
            description=(
                "Gazebo-only initial map->odom y [m]. For rmuc_2025 this is "
                "the robot spawn y plus the PGM origin y."
            ),
        ),
        DeclareLaunchArgument("initial_map_to_odom_z", default_value="0.0"),
        DeclareLaunchArgument("initial_map_to_odom_roll", default_value="0.0"),
        DeclareLaunchArgument("initial_map_to_odom_pitch", default_value="0.0"),
        DeclareLaunchArgument("initial_map_to_odom_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument(
            "use_viewer",
            default_value="false",
            description="Open the Gazebo GUI; ignored when headless is true",
        ),
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run Gazebo without any GUI. Default for regression runs",
        ),
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="true",
            description=(
                "Use Gazebo's explicit off-screen server rendering path when "
                "headless. This preserves GPU LiDAR sensor output without a GUI."
            ),
        ),
        DeclareLaunchArgument(
            "launch_nav2",
            default_value="false",
            description=(
                "Kept for interface compatibility only. This profile never starts "
                "Navigation2; planning is JPS + MINCO + ats_swerve_mpc."
            ),
        ),
        DeclareLaunchArgument(
            "planning_grid_owner",
            default_value="rog_map",
            description="Single owner of /rc_esdf/planning_grid: rog_map or rc_esdf",
        ),
        DeclareLaunchArgument(
            "robot_description_package", default_value="ats_robot_description"
        ),
        DeclareLaunchArgument(
            "robot_description_xmacro", default_value="ats_sentry_robot.sdf.xmacro"
        ),
        DeclareLaunchArgument(
            "solver_mode",
            default_value="ilqr",
            description="ats_swerve_mpc solver mode; qp_shadow is diagnostic-only",
        ),
        DeclareLaunchArgument(
            "require_gimbal_status",
            default_value="false",
            description=(
                "The Gazebo profile has no serial gimbal bridge, so /gimbal/yaw_status "
                "is never published here. Keep false in simulation; the real vehicle "
                "keeps the gate enabled through node_params.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "projection_rate_hz",
            default_value="2.0",
            description=(
                "ROGMap projection publication rate for this Gazebo profile. "
                "It must remain above 1 Hz because ats_goal_manager keeps a "
                "one-second immutable planning-snapshot lease."
            ),
        ),
        DeclareLaunchArgument(
            "enable_test_fault_injection",
            default_value="false",
            description=(
                "Startup-only authorization for ROGMap/adapter fault fixtures. "
                "Only an isolated fault run may set it true."
            ),
        ),
        DeclareLaunchArgument(
            "enable_camera_sensors",
            default_value="false",
            description=(
                "Keep the industrial-camera <sensor> in the spawned robot. False "
                "here: the camera and the mid360 gpu_lidar share one Sensors "
                "render thread, and measurements on this world (RTF pinned at "
                "1.0) put the lidar at 27.3 Hz alone against 12.5 Hz with the "
                "1920x1080@30Hz camera present, which starves Point-LIO."
            ),
        ),
        DeclareLaunchArgument(
            "use_direct_gazebo_lidar_bridge",
            default_value="false",
            description=(
                "Replace only the Mid360 generic GZ-to-ROS mapping with the "
                "dedicated BEST_EFFORT direct bridge."
            ),
        ),
        DeclareLaunchArgument(
            "point_lio_scan_line",
            default_value="32",
            description=(
                "Gazebo Mid360 vertical line count. Must match the 32-row "
                "PointCloud2 emitted by the robot description."
            ),
        ),
        DeclareLaunchArgument(
            "livox_update_rate_hz",
            default_value="10.0",
            description=(
                "Gazebo Mid360 update rate. Its reciprocal is used for both "
                "the bridge offset-time span and Point-LIO measurement window."
            ),
        ),
        DeclareLaunchArgument(
            "livox_horizontal_samples",
            default_value="625",
            description=(
                "Gazebo Mid360 horizontal ray count at fixed 10 Hz and 32 rings. "
                "The navigation default is 625 (200 k rays/s)."
            ),
        ),
        DeclareLaunchArgument(
            "lidar_bridge_publisher_depth",
            default_value="10",
            description=(
                "Generic Mid360 bridge ROS publisher queue depth. The default "
                "preserves the existing reliable KeepLast(10) contract."
            ),
        ),
        DeclareLaunchArgument(
            "lidar_bridge_publisher_reliability",
            default_value="reliable",
            description=(
                "Generic Mid360 bridge ROS publisher reliability: reliable or "
                "best_effort."
            ),
        ),
        DeclareLaunchArgument("launch_terrain_analysis", default_value="true"),
        DeclareLaunchArgument("launch_executed_path_observer", default_value="true"),
        DeclareLaunchArgument("nav_start_delay_sec", default_value="6.0"),
        DeclareLaunchArgument("rog_map_start_delay_sec", default_value="10.0"),
        DeclareLaunchArgument("rviz_delay_sec", default_value="8.0"),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=os.path.join(pkg_simulator, "rviz", "ats_gazebo_nav.rviz"),
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
        # Livox mounting pose on the big-yaw frame. Must stay numerically equal
        # to the xmacro block in ats_sentry_robot.sdf.xmacro, otherwise the
        # point cloud and the TF tree disagree.
        DeclareLaunchArgument("lidar_static_tf_x", default_value="-0.2"),
        DeclareLaunchArgument("lidar_static_tf_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_z", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_roll", default_value="0.0"),
        DeclareLaunchArgument("lidar_static_tf_pitch", default_value="0.0"),
        DeclareLaunchArgument(
            "lidar_static_tf_yaw", default_value="-1.0646508437165408"
        ),
    ]

    log_level = LaunchConfiguration("log_level")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "world_sdf_path": LaunchConfiguration("world_sdf_path"),
            "headless": LaunchConfiguration("headless"),
            "headless_rendering": LaunchConfiguration("headless_rendering"),
            "use_viewer": LaunchConfiguration("use_viewer"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    spawn_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "world": world,
            "use_sim_time": use_sim_time,
            "robot_description_package": LaunchConfiguration(
                "robot_description_package"
            ),
            "robot_description_xmacro": LaunchConfiguration("robot_description_xmacro"),
            # Never true here: it would add a second chassis command publisher.
            "launch_robot_base": "false",
            "enable_camera_sensors": LaunchConfiguration("enable_camera_sensors"),
            "use_direct_gazebo_lidar_bridge": LaunchConfiguration(
                "use_direct_gazebo_lidar_bridge"
            ),
            "livox_update_rate_hz": livox_update_rate_hz,
            "livox_horizontal_samples": LaunchConfiguration("livox_horizontal_samples"),
            "lidar_bridge_publisher_depth": LaunchConfiguration(
                "lidar_bridge_publisher_depth"
            ),
            "lidar_bridge_publisher_reliability": LaunchConfiguration(
                "lidar_bridge_publisher_reliability"
            ),
        }.items(),
    )

    lidar_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_gimbal_yaw_odom_to_front_mid360",
        output="screen",
        arguments=[
            "--x",
            LaunchConfiguration("lidar_static_tf_x"),
            "--y",
            LaunchConfiguration("lidar_static_tf_y"),
            "--z",
            LaunchConfiguration("lidar_static_tf_z"),
            "--roll",
            LaunchConfiguration("lidar_static_tf_roll"),
            "--pitch",
            LaunchConfiguration("lidar_static_tf_pitch"),
            "--yaw",
            LaunchConfiguration("lidar_static_tf_yaw"),
            "--frame-id",
            "gimbal_yaw_odom",
            "--child-frame-id",
            "front_mid360",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    livox_bridge = Node(
        package="rmu_gazebo_simulator",
        executable="gz_livox_bridge_node",
        name="gz_livox_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_cloud_topic": ["/", robot_name, "/livox/lidar"],
                "input_imu_topic": ["/", robot_name, "/livox/imu"],
                "output_cloud_topic": "/livox/lidar",
                "output_imu_topic": "/livox/imu",
                "lidar_frame_id": "front_mid360",
                "imu_frame_id": "front_mid360",
                # The SDF update rate, synthetic CustomPoint offsets and
                # Point-LIO measurement window share one period. A timing A/B
                # cannot silently make deskew offsets describe another sensor.
                "scan_period_sec": ParameterValue(
                    PythonExpression(["1.0 / float('", livox_update_rate_hz, "')"]),
                    value_type=float,
                ),
            }
        ],
    )

    point_lio = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "common.lid_topic": "/livox/lidar",
                "common.imu_topic": "/livox/imu",
                "preprocess.scan_line": ParameterValue(
                    point_lio_scan_line, value_type=int
                ),
                "mapping.lidar_time_inte": ParameterValue(
                    PythonExpression(["1.0 / float('", livox_update_rate_hz, "')"]),
                    value_type=float,
                ),
                # The Gazebo IMU and the bridged LiDAR both use
                # front_mid360.  The root parameter file contains calibrated
                # real-vehicle gravity and IMU-to-LiDAR extrinsics; applying
                # them to this coincident simulated frame tilts the stationary
                # map and turns the robot's start cell into terrain obstacle.
                # Keep this override local to the simulation profile.
                "mapping.gravity": [0.0, 0.0, -9.81],
                "mapping.gravity_init": [0.0, 0.0, -9.81],
                "mapping.extrinsic_T": [0.0, 0.0, 0.0],
                "mapping.extrinsic_R": [
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0,
                    0.0,
                    1.0,
                ],
                "prior_pcd.enable": False,
                "prior_pcd.prior_pcd_map_path": "",
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    loam_interface = Node(
        package="loam_interface",
        executable="loam_interface_node",
        name="loam_interface",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Single publisher of odom->gimbal_yaw_odom.
    #
    # base_frame is deliberately empty here. On the real vehicle the chassis
    # footprint frame comes from robot_state_publisher, which this profile runs
    # under the robot namespace with /tf remapped to <robot>/tf, so
    # front_mid360 -> base_footprint never appears on the global /tf. The node
    # needs that edge as an *input* before it will publish anything, so leaving
    # base_frame set would gate /odometry - and the whole localization chain -
    # on a frame that no node in this profile reads. A static TF is not a valid
    # substitute either: gimbal_yaw_odom and base_footprint are separated by the
    # revolute big-yaw joint, so a fixed value would be wrong the moment the
    # gimbal turns.
    sensor_scan = Node(
        package="sensor_scan_generation",
        executable="sensor_scan_generation_node",
        name="sensor_scan_generation",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "base_frame": "",
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    static_map = Node(
        package="ats_nav_bringup",
        executable="static_map_publisher.py",
        name="static_map_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_yaml_file": map_yaml,
                "map_topic": "/map",
                "frame_id": "map",
            }
        ],
    )

    # Single publisher of map->odom and of /localization.
    localization_fusion = Node(
        package="small_gicp_relocalization",
        executable="localization_fusion_node",
        name="localization_fusion",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/odometry",
                "localization_topic": "/localization",
                "status_topic": "/localization/status",
                "map_frame": "map",
                "odom_frame": "odom",
                "robot_base_frame": "gimbal_yaw_odom",
                "publish_tf": True,
                # This is a fixed static-PGM-to-Gazebo registration, not a
                # ground-truth feedback path. Point-LIO remains the source of
                # odometry and /localization observations.
                "allow_initial_identity": False,
                "use_initial_map_to_odom": True,
                "initial_map_to_odom_x": ParameterValue(
                    initial_map_to_odom_x, value_type=float
                ),
                "initial_map_to_odom_y": ParameterValue(
                    initial_map_to_odom_y, value_type=float
                ),
                "initial_map_to_odom_z": ParameterValue(
                    initial_map_to_odom_z, value_type=float
                ),
                "initial_map_to_odom_roll": ParameterValue(
                    initial_map_to_odom_roll, value_type=float
                ),
                "initial_map_to_odom_pitch": ParameterValue(
                    initial_map_to_odom_pitch, value_type=float
                ),
                "initial_map_to_odom_yaw": ParameterValue(
                    initial_map_to_odom_yaw, value_type=float
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    terrain = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        condition=IfCondition(LaunchConfiguration("launch_terrain_analysis")),
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    terrain_ext = Node(
        package="terrain_analysis_ext",
        executable="terrainAnalysisExt",
        name="terrain_analysis_ext",
        condition=IfCondition(LaunchConfiguration("launch_terrain_analysis")),
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )

    rog_map = Node(
        package="ats_rog_map",
        executable="ats_rog_map_node",
        name="ats_rog_map",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "enable_test_fault_injection": ParameterValue(
                    enable_test_fault_injection, value_type=bool
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    rog_map_adapter = Node(
        package="ats_rog_map_adapter",
        executable="ats_rog_map_adapter_node",
        name="ats_rog_map_adapter",
        condition=rog_map_owned,
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "planning_grid_owner": planning_grid_owner,
                "require_localization_status": True,
                # The shared real-vehicle profile projects at 0.5 Hz for its
                # measured CPU budget.  Gazebo completes this request in
                # milliseconds while the goal manager keeps a 1 s immutable
                # snapshot lease, so this profile must refresh above 1 Hz
                # rather than weakening that safety lease.
                "projection_rate_hz": ParameterValue(
                    projection_rate_hz, value_type=float
                ),
                "enable_test_fault_injection": ParameterValue(
                    enable_test_fault_injection, value_type=bool
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    goal_manager = Node(
        package="ats_goal_manager",
        executable="ats_goal_manager_node",
        name="ats_goal_manager",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "require_localization_status": True,
                # There is no serial gimbal-status producer in Gazebo.  The
                # chassis adapter uses the simulated joint state for the yaw
                # transform, while real-vehicle launches retain the ACK lease.
                "require_gimbal_status": ParameterValue(
                    require_gimbal_status, value_type=bool
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    minco = Node(
        package="minco_planner",
        executable="minco_planner_node",
        name="minco_planner",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        arguments=["--ros-args", "--log-level", log_level],
    )

    mpc = Node(
        package="ats_swerve_mpc",
        executable="ats_swerve_mpc_node",
        name="ats_swerve_mpc",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "command_topic": "/cmd_vel_mpc",
                "require_localization_status": True,
                "require_gimbal_status": ParameterValue(
                    require_gimbal_status, value_type=bool
                ),
                "solver_mode": LaunchConfiguration("solver_mode"),
                "publish_debug_paths": True,
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # Single owner between MPC and the Gazebo chassis.
    chassis_adapter = Node(
        package="rmu_gazebo_simulator",
        executable="chassis_cmd_adapter.py",
        name="gz_chassis_cmd_adapter",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/cmd_vel_mpc",
                "motion_control_topic": "/motion_control",
                "chassis_topic": ["/", robot_name, "/cmd_vel"],
                "joint_state_topic": ["/", robot_name, "/joint_states"],
                "emergency_stop_topic": "/planner/emergency_stop",
                "big_yaw_joint_name": "gimbal_yaw_odom_joint",
            }
        ],
    )

    executed_path = Node(
        package="rmu_gazebo_simulator",
        executable="executed_path_observer.py",
        name="executed_path_observer",
        condition=IfCondition(LaunchConfiguration("launch_executed_path_observer")),
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/localization",
                "output_topic": "/ats_swerve_mpc/executed_path",
                "reference_path_topic": "/minco/reference_path",
                "emergency_stop_topic": "/planner/emergency_stop",
            }
        ],
    )

    rviz = TimerAction(
        period=LaunchConfiguration("rviz_delay_sec"),
        actions=[
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                name="ats_gazebo_nav_rviz2",
                output="screen",
                # Keep an explicit title so the regression can capture the
                # correct RViz window without depending on RViz2's versioned
                # default title format. Software GL affects RViz only; Gazebo
                # remains headless and does not share this render context.
                arguments=[
                    "--display-title-format",
                    "ATS Gazebo Navigation - RViz",
                    "-d",
                    LaunchConfiguration("rviz_config_file"),
                ],
                additional_env={"LIBGL_ALWAYS_SOFTWARE": "1"},
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ],
    )

    localization_group = TimerAction(
        period=LaunchConfiguration("nav_start_delay_sec"),
        actions=[
            point_lio,
            loam_interface,
            sensor_scan,
            localization_fusion,
            terrain,
            terrain_ext,
        ],
    )
    planning_group = TimerAction(
        period=LaunchConfiguration("rog_map_start_delay_sec"),
        actions=[
            rog_map,
            rog_map_adapter,
            goal_manager,
            minco,
            mpc,
            chassis_adapter,
            executed_path,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"))
    ld.add_action(SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"))
    for declaration in declarations:
        ld.add_action(declaration)
    for action in (
        gazebo,
        spawn_robots,
        lidar_static_tf,
        livox_bridge,
        static_map,
        localization_group,
        planning_group,
        rviz,
    ):
        ld.add_action(action)
    return ld
