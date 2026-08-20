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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes", "on")


def launch_setup(context: LaunchContext) -> list:
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    def resolve(name: str) -> str:
        return context.perform_substitution(LaunchConfiguration(name))

    world = resolve("world")
    world_sdf_path = resolve("world_sdf_path") or os.path.join(
        pkg_simulator, "resource", "worlds", f"{world}_world.sdf"
    )
    if not os.path.isfile(world_sdf_path):
        raise RuntimeError(f"World SDF not found: {world_sdf_path}")

    ign_config_path = resolve("ign_config_path") or os.path.join(
        pkg_simulator, "resource", "ign", "gui.config"
    )

    headless = _as_bool(resolve("headless"))
    use_viewer = _as_bool(resolve("use_viewer"))
    use_sim_time = _as_bool(resolve("use_sim_time"))
    headless_rendering = _as_bool(resolve("headless_rendering"))

    # headless wins over use_viewer: a CI/regression run must never open a GUI
    # just because a profile forgot to clear use_viewer.
    show_gui = use_viewer and not headless

    # `-r` starts the simulation immediately. Without it the world stays paused
    # until someone presses play in the GUI, which in headless mode means the
    # clock never advances and every downstream freshness check fails.
    gz_args = ["-r", " ", world_sdf_path]
    if show_gui:
        gz_args += [" --gui-config ", ign_config_path]
    else:
        gz_args.insert(0, "-s ")
        # GPU LiDAR remains a render workload even without a GUI. Select
        # Gazebo's explicit off-screen server-rendering path for headless runs.
        if headless_rendering:
            gz_args.insert(0, "--headless-rendering ")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "6",
            "gz_args": "".join(gz_args),
        }.items(),
    )

    # The relay is the only ROS /clock publisher. It subscribes to Gazebo
    # transport directly, so no unthrottled ROS intermediate topic exists.
    clock_relay = Node(
        package="rmu_gazebo_simulator",
        executable="gz_clock_relay_node",
        name="gz_clock_relay",
        parameters=[
            {
                # A clock source cannot itself depend on /clock.
                "use_sim_time": False,
                "gazebo_clock_topic": "/clock",
                "ros_clock_topic": "/clock",
                "max_publish_rate_hz": ParameterValue(
                    LaunchConfiguration("clock_publish_rate_hz"), value_type=float
                ),
            }
        ],
    )

    return [gazebo, clock_relay]


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "world",
            default_value="rmuc_2025",
            description="World name resolved to resource/worlds/<world>_world.sdf",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_sdf_path",
            default_value="",
            description="Explicit world SDF path. Empty resolves from 'world'",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "ign_config_path",
            default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
            description="Path to the Ignition Gazebo GUI configuration file",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "headless",
            default_value="true",
            description="Run the server without any GUI. Overrides use_viewer",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_viewer",
            default_value="false",
            description="Open the Gazebo GUI. Ignored when headless is true",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "headless_rendering",
            default_value="true",
            description=(
                "Use Gazebo's explicit off-screen server rendering path when "
                "headless. Ignored when a GUI viewer is enabled."
            ),
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
            "clock_publish_rate_hz",
            default_value="100.0",
            description=(
                "Maximum ROS /clock rate. Gazebo time values are unchanged; "
                "100 Hz bounds DDS fan-out while exceeding the 50 Hz chassis loop"
            ),
        )
    )
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
