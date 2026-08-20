# Copyright 2025 Lihan Chen
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

import math
import os
import re

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


_CAMERA_SENSOR_RE = re.compile(
    r"[ \t]*<sensor name=\"[^\"]*\" type=\"camera\">[\s\S]*?</sensor>\n?"
)


def _strip_camera_sensors(sdf_xml: str) -> tuple:
    """Remove ``type="camera"`` sensors, keeping their links and joints.

    The Gazebo server runs one Sensors render thread for every rendering
    sensor, and it runs even with ``-s`` headless. Measured on this world with
    RTF pinned at 1.0, the mid360 gpu_lidar alone reaches 27.3 Hz, two lidars
    reach 22.0 Hz each, and adding one 1920x1080@30Hz industrial camera drops
    the lidar of that same robot to 12.5 Hz. Point-LIO then falls far below the
    0.5 s odometry freshness window that localization_fusion enforces against
    the wall clock, so /localization/status never reaches TRACKING and the
    planning-grid adapter stays ready=false.

    Only the ``<sensor>`` element is dropped. The camera link, its joint and
    the optical frame stay, so the URDF and the TF tree are byte-identical to
    the camera-enabled build and no navigation frame disappears.
    """
    stripped, count = _CAMERA_SENSOR_RE.subn("", sdf_xml)
    return stripped, count


def launch_setup(context: LaunchContext) -> list:
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    def resolve(name: str) -> str:
        return context.perform_substitution(LaunchConfiguration(name))

    description_package = resolve("robot_description_package")
    description_xmacro = resolve("robot_description_xmacro")
    use_sim_time = _as_bool(resolve("use_sim_time"))
    launch_robot_base = _as_bool(resolve("launch_robot_base"))
    enable_camera_sensors = _as_bool(resolve("enable_camera_sensors"))
    use_direct_gazebo_lidar_bridge = _as_bool(resolve("use_direct_gazebo_lidar_bridge"))
    livox_update_rate_hz = resolve("livox_update_rate_hz")
    livox_horizontal_samples = resolve("livox_horizontal_samples")
    lidar_bridge_publisher_depth = resolve("lidar_bridge_publisher_depth")
    lidar_bridge_publisher_reliability = (
        resolve("lidar_bridge_publisher_reliability").strip().lower()
    )
    try:
        livox_update_rate_hz_value = float(livox_update_rate_hz)
    except ValueError as error:
        raise RuntimeError(
            "livox_update_rate_hz must be a finite positive number, got "
            f"'{livox_update_rate_hz}'"
        ) from error
    if (
        not math.isfinite(livox_update_rate_hz_value)
        or livox_update_rate_hz_value <= 0.0
    ):
        raise RuntimeError(
            "livox_update_rate_hz must be a finite positive number, got "
            f"'{livox_update_rate_hz}'"
        )
    try:
        livox_horizontal_samples_value = int(livox_horizontal_samples)
    except ValueError as error:
        raise RuntimeError(
            "livox_horizontal_samples must be a positive integer, got "
            f"'{livox_horizontal_samples}'"
        ) from error
    if livox_horizontal_samples_value <= 0:
        raise RuntimeError(
            "livox_horizontal_samples must be a positive integer, got "
            f"'{livox_horizontal_samples}'"
        )
    try:
        lidar_bridge_publisher_depth_value = int(lidar_bridge_publisher_depth)
    except ValueError as error:
        raise RuntimeError(
            "lidar_bridge_publisher_depth must be a positive integer, got "
            f"'{lidar_bridge_publisher_depth}'"
        ) from error
    if lidar_bridge_publisher_depth_value <= 0:
        raise RuntimeError(
            "lidar_bridge_publisher_depth must be a positive integer, got "
            f"'{lidar_bridge_publisher_depth}'"
        )
    if lidar_bridge_publisher_reliability not in ("reliable", "best_effort"):
        raise RuntimeError(
            "lidar_bridge_publisher_reliability must be reliable or best_effort, got "
            f"'{lidar_bridge_publisher_reliability}'"
        )

    robot_xmacro_path = os.path.join(
        get_package_share_directory(description_package),
        "resource",
        "xmacro",
        description_xmacro,
    )
    if not os.path.isfile(robot_xmacro_path):
        raise RuntimeError(
            f"Robot xmacro not found: {robot_xmacro_path}. Check "
            "robot_description_package/robot_description_xmacro."
        )

    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # Get spawn robot init pose
    gz_world_path = resolve("gz_world_path") or os.path.join(
        pkg_simulator, "config", "gz_world.yaml"
    )
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)

    selected_world = resolve("world") or config.get("world")
    robots = config.get("robots", {}).get(selected_world)
    if not robots:
        raise RuntimeError(
            f"No robot spawn entry for world '{selected_world}' in {gz_world_path}."
        )

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    actions = []

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate(
            {
                "global_initial_color": robot["color"],
                "nav_livox_update_rate_hz": livox_update_rate_hz_value,
                "nav_livox_horizontal_samples": livox_horizontal_samples_value,
            }
        )
        robot_xml = xmacro.to_string()

        # Generate URDF from SDF. This uses the camera-enabled SDF on purpose:
        # only <sensor> elements are stripped below, so the URDF is the same
        # either way, and deriving it here keeps that independence explicit.
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        if not enable_camera_sensors:
            robot_xml, stripped_cameras = _strip_camera_sensors(robot_xml)
            print(
                f"[spawn_robots] {robot['name']}: stripped {stripped_cameras} "
                "camera sensor(s); rendering budget reserved for the lidar."
            )

        # replace the <robot_name> in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={
                "<robot_name>": robot["name"],
                "<direct_gazebo_lidar_bridge_prefix>": (
                    "#" if use_direct_gazebo_lidar_bridge else ""
                ),
            },
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        # rmoss_gz_base/rmua19_robot_base owns a chassis follow-yaw controller
        # that publishes its own chassis command. In the ATS profile the single
        # owner between MPC and the chassis is gz_chassis_cmd_adapter, so this
        # node must stay disabled there; it is only kept for the standalone
        # referee/teleop demo profile.
        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[
                robot_config,
                {"robot_name": robot["name"], "use_sim_time": use_sim_time},
            ],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        bridge_parameters = {
            "config_file": aft_replace_ros_bridge_params,
            "use_sim_time": use_sim_time,
        }
        if not use_direct_gazebo_lidar_bridge:
            lidar_topic = f"/{robot['name']}/livox/lidar"
            bridge_parameters[
                f"qos_overrides.{lidar_topic}.publisher.depth"
            ] = lidar_bridge_publisher_depth_value
            bridge_parameters[
                f"qos_overrides.{lidar_topic}.publisher.reliability"
            ] = lidar_bridge_publisher_reliability

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[bridge_parameters],
        )

        direct_lidar_bridge = Node(
            package="rmu_gazebo_simulator",
            executable="gz_lidar_ros_bridge_node",
            name="gz_lidar_ros_bridge",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "gazebo_lidar_topic": (
                        f"/world/default/model/{robot['name']}/link/front_mid360/"
                        "sensor/front_mid360_lidar/scan/points"
                    ),
                    "ros_lidar_topic": f"/{robot['name']}/livox/lidar",
                    "publisher_depth": 1,
                }
            ],
        )

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        actions.append(spawn_robot)
        if launch_robot_base:
            actions.append(robot_base)
        actions.append(robot_state_publisher)
        actions.append(robot_ign_bridge)
        if use_direct_gazebo_lidar_bridge:
            actions.append(direct_lidar_bridge)
        actions.append(set_performer_service)

    return actions


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="rmuc_2025",
            description="Name of the Gazebo world whose spawn entry is used",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "gz_world_path",
            default_value="",
            description="Path to gz_world.yaml. Empty falls back to the package config",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_description_package",
            default_value="ats_robot_description",
            description="Package owning the robot xmacro/mesh resources",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "robot_description_xmacro",
            default_value="ats_sentry_robot.sdf.xmacro",
            description="Robot xmacro file name inside <package>/resource/xmacro",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use the Gazebo /clock as ROS time source",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "launch_robot_base",
            default_value="false",
            description=(
                "Start rmoss_gz_base/rmua19_robot_base. Must stay false in the ATS "
                "profile: it would become a second chassis command publisher"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "enable_camera_sensors",
            default_value="false",
            description=(
                "Keep the industrial-camera <sensor> in the spawned SDF. False in "
                "the navigation profile: the camera shares the single Sensors "
                "render thread with the mid360 gpu_lidar and roughly halves its "
                "measured rate, which starves Point-LIO. Links, joints and the "
                "URDF are unaffected either way"
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_direct_gazebo_lidar_bridge",
            default_value="false",
            description=(
                "Use the dedicated BEST_EFFORT Mid360 GZ-to-ROS bridge and "
                "disable the matching generic parameter_bridge mapping."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "livox_update_rate_hz",
            default_value="10.0",
            description=(
                "Gazebo Mid360 update rate. The top-level launch derives bridge "
                "and Point-LIO scan periods from this same value."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "livox_horizontal_samples",
            default_value="625",
            description=(
                "Gazebo Mid360 horizontal ray count at fixed 10 Hz and 32 rings. "
                "The navigation default is 625 (200 k rays/s)."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "lidar_bridge_publisher_depth",
            default_value="10",
            description=(
                "ROS publisher depth for the generic Mid360 bridge. This is "
                "ignored when the direct bridge path is selected."
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "lidar_bridge_publisher_reliability",
            default_value="reliable",
            description=(
                "ROS publisher reliability for the generic Mid360 bridge: "
                "reliable or best_effort."
            ),
        )
    )
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
