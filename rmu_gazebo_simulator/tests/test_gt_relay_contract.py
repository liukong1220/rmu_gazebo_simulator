#!/usr/bin/env python3
# Copyright 2026 ATS Sentry Navigation Contributors
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

"""Behavioral regression tests for GT relay shutdown and cloud QoS contracts."""

import importlib.util
import math
from types import SimpleNamespace
from pathlib import Path
import unittest
from unittest.mock import patch
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.executors import ExternalShutdownException
from rclpy.qos import ReliabilityPolicy


BRIDGE_DIR = Path(__file__).resolve().parents[1] / "scripts" / "ats_bridge"
REFEREE_ENTRYPOINT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "referee_system"
    / "simple_competition_1v1.py"
)


def load_relay(name):
    spec = importlib.util.spec_from_file_location(name, BRIDGE_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ODOM = load_relay("gazebo_gt_odometry_relay")
SCAN = load_relay("gazebo_gt_registered_scan_relay")


class RelayShutdownTest(unittest.TestCase):
    def exercise_main(self, module, constructor, error):
        with patch.object(module.rclpy, "init"), \
                patch.object(module, constructor) as node_type, \
                patch.object(module.rclpy, "spin", side_effect=error), \
                patch.object(module.rclpy, "try_shutdown") as shutdown:
            if isinstance(error, RuntimeError):
                with self.assertRaisesRegex(RuntimeError, "active callback failure"):
                    module.main()
            else:
                module.main()
            node_type.return_value.destroy_node.assert_called_once_with()
            shutdown.assert_called_once_with()

    def test_expected_shutdown_is_owned_and_cleanup_is_idempotent(self):
        for module, constructor in (
            (ODOM, "GazeboGtOdometryRelay"),
            (SCAN, "GazeboGtRegisteredScanRelay"),
        ):
            for error in (KeyboardInterrupt(), ExternalShutdownException()):
                with self.subTest(relay=constructor, error=type(error).__name__):
                    self.exercise_main(module, constructor, error)

    def test_active_callback_failure_propagates(self):
        for module, constructor in (
            (ODOM, "GazeboGtOdometryRelay"),
            (SCAN, "GazeboGtRegisteredScanRelay"),
        ):
            with self.subTest(relay=constructor):
                    self.exercise_main(module, constructor, RuntimeError("active callback failure"))


class GazeboEntrypointTest(unittest.TestCase):
    def test_referee_script_is_directly_executable_by_ros_launch(self):
        self.assertTrue(
            REFEREE_ENTRYPOINT.read_bytes().startswith(b"#!/usr/bin/env python3\n")
        )

    def test_referee_main_owns_external_shutdown_cleanup(self):
        source = REFEREE_ENTRYPOINT.read_text(encoding="utf-8")
        self.assertIn("ExternalShutdownException", source)
        self.assertIn("except (KeyboardInterrupt, ExternalShutdownException):", source)
        self.assertIn("rclpy.try_shutdown()", source)


class RegisteredScanQoSTest(unittest.TestCase):
    def test_generic_and_direct_inputs_keep_registered_output_reliable(self):
        for requested, expected in (
            ("reliable", ReliabilityPolicy.RELIABLE),
            ("best_effort", ReliabilityPolicy.BEST_EFFORT),
        ):
            with self.subTest(input_reliability=requested):
                rclpy.init(args=["--ros-args", "-p", f"input_reliability:={requested}"])
                node = None
                try:
                    node = SCAN.GazeboGtRegisteredScanRelay()
                    inputs = [sub for sub in node.subscriptions
                              if sub.topic_name == node.input_topic]
                    self.assertEqual(len(inputs), 1)
                    self.assertEqual(inputs[0].qos_profile.reliability, expected)
                    self.assertEqual(node._pub.qos_profile.reliability,
                                     ReliabilityPolicy.RELIABLE)
                finally:
                    if node is not None:
                        node.destroy_node()
                    rclpy.try_shutdown()


class GimbalOdometryContractTest(unittest.TestCase):
    def relay(self):
        return SimpleNamespace(
            gt_child_frame="robot/gimbal_yaw_odom", odom_frame="odom",
            base_frame="gimbal_yaw_odom", spawn_x=4.0, spawn_y=9.0,
            spawn_yaw=math.pi / 2.0, pub=Mock(), tf_broadcaster=Mock(),
            get_logger=Mock(return_value=Mock()),
        )

    def test_native_gimbal_pose_keeps_roll_pitch_height_and_child_twist(self):
        relay = self.relay()
        msg = Odometry()
        msg.header.frame_id = "world"
        msg.header.stamp.sec = 23
        msg.child_frame_id = "robot/gimbal_yaw_odom"
        msg.pose.pose.position.x = 5.0
        msg.pose.pose.position.y = 11.0
        msg.pose.pose.position.z = 0.2
        # Native orientation Rz(spawn_yaw) * Rx(roll); normalization must
        # retain Rx(roll), not flatten the link to chassis planar yaw.
        roll = 0.3
        s, c = math.sin(roll / 2.0), math.cos(roll / 2.0)
        k = math.sqrt(0.5)
        msg.pose.pose.orientation.x = k * s
        msg.pose.pose.orientation.y = k * s
        msg.pose.pose.orientation.z = k * c
        msg.pose.pose.orientation.w = k * c
        msg.twist.twist.linear.x = 0.7
        msg.twist.twist.linear.y = -0.4
        msg.twist.twist.angular.z = 1.2

        ODOM.GazeboGtOdometryRelay._on_gt(relay, msg)

        out = relay.pub.publish.call_args.args[0]
        self.assertEqual(out.header.stamp, msg.header.stamp)
        self.assertEqual(out.child_frame_id, "gimbal_yaw_odom")
        self.assertAlmostEqual(out.pose.pose.position.x, 2.0)
        self.assertAlmostEqual(out.pose.pose.position.y, -1.0)
        self.assertAlmostEqual(out.pose.pose.position.z, 0.2)
        for actual, expected in zip(
            (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
             out.pose.pose.orientation.z, out.pose.pose.orientation.w),
            (s, 0.0, 0.0, c),
        ):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(out.twist, msg.twist)
        tf = relay.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(tf.header, out.header)
        self.assertEqual(tf.transform.rotation, out.pose.pose.orientation)

    def test_chassis_or_wrong_parent_is_rejected_without_tf(self):
        for parent, child in (("world", "robot/chassis"),
                              ("robot/odom", "robot/gimbal_yaw_odom")):
            with self.subTest(parent=parent, child=child):
                relay = self.relay()
                msg = Odometry()
                msg.header.frame_id = parent
                msg.child_frame_id = child
                ODOM.GazeboGtOdometryRelay._on_gt(relay, msg)
                relay.pub.publish.assert_not_called()
                relay.tf_broadcaster.sendTransform.assert_not_called()


class RegisteredScanTransformTest(unittest.TestCase):
    def test_sensor_points_are_transformed_not_relabelled(self):
        header = Header(frame_id="front_mid360")
        header.stamp.sec = 17
        cloud = point_cloud2.create_cloud_xyz32(header, [(1.0, 0.0, 0.5)])
        transform = TransformStamped()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "front_mid360"
        transform.transform.translation.x = 3.0
        transform.transform.translation.y = -2.0
        transform.transform.rotation.z = math.sin(math.pi / 4.0)
        transform.transform.rotation.w = math.cos(math.pi / 4.0)

        result = SCAN._transform_xyz_cloud(cloud, transform)

        points = list(point_cloud2.read_points(result, field_names=("x", "y", "z")))
        self.assertEqual(result.header.frame_id, "odom")
        self.assertEqual(result.header.stamp, header.stamp)
        self.assertEqual(len(points), 1)
        for actual, expected in zip(points[0], (3.0, -1.0, 0.5)):
            self.assertAlmostEqual(float(actual), expected)


if __name__ == "__main__":
    unittest.main()
