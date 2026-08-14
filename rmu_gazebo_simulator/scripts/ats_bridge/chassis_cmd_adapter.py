#!/usr/bin/env python3
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

"""Single owner of the Gazebo chassis command for the ATS navigation chain.

Ownership contract (must not be duplicated by any other node):

* ``/cmd_vel_mpc``      : subscribed here, published only by ``ats_swerve_mpc``.
* ``/motion_control``   : published here and nowhere else in the Gazebo profile.
* ``<chassis_topic>``   : published here, bridged ROS -> GZ into the
  ``SwerveDrive4WS`` plugin of the spawned robot.

Semantics:

* input  ``geometry_msgs/Twist`` in the ``gimbal_yaw_odom`` body frame,
  ``[vx, vy, wz]`` in m/s and rad/s;
* output ``manda_can_control/MotionCtrl`` with the same body-frame semantics,
  plus a ``geometry_msgs/Twist`` in the *chassis* frame for Gazebo.

The two frames differ by the big-yaw joint angle. The chassis-frame command is
``v_chassis = R(psi) * v_gimbal_yaw_odom``. When the big-yaw feedback is missing
the node publishes zero instead of passing the command straight through: a
silent pass-through would rotate the commanded direction by psi, which is a
"command direction does not match motion" fault, not a degraded mode.

Zero-velocity fallbacks (all produce an exact zero on both outputs):

* no ``/cmd_vel_mpc`` within ``command_timeout``;
* ``/planner/emergency_stop`` latched true;
* missing big-yaw feedback while ``require_big_yaw_feedback`` is true.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from manda_can_control.msg import MotionCtrl
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class ChassisCmdAdapter(Node):
    """Bridge ``/cmd_vel_mpc`` to ``/motion_control`` and the Gazebo chassis."""

    def __init__(self) -> None:
        super().__init__("gz_chassis_cmd_adapter")

        self.declare_parameter("input_topic", "/cmd_vel_mpc")
        self.declare_parameter("motion_control_topic", "/motion_control")
        self.declare_parameter("chassis_topic", "/red_standard_robot1/cmd_vel")
        self.declare_parameter("joint_state_topic", "/red_standard_robot1/joint_states")
        self.declare_parameter("emergency_stop_topic", "/planner/emergency_stop")
        self.declare_parameter("big_yaw_joint_name", "gimbal_yaw_odom_joint")
        self.declare_parameter("transform_linear_with_big_yaw", True)
        self.declare_parameter("require_big_yaw_feedback", True)
        # Saturation must never be looser than the MPC feasible set, otherwise
        # "MPC limit == chassis limit" stops holding and the zero-command
        # derivation loses its upper bound.
        self.declare_parameter("max_linear_x", 1.5)
        self.declare_parameter("max_linear_y", 1.5)
        self.declare_parameter("max_angular_z", 2.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("joint_state_timeout", 0.5)
        self.declare_parameter("publish_rate_hz", 50.0)

        self.big_yaw_joint_name = str(self.get_parameter("big_yaw_joint_name").value)
        self.transform_with_big_yaw = bool(
            self.get_parameter("transform_linear_with_big_yaw").value
        )
        self.require_big_yaw = bool(
            self.get_parameter("require_big_yaw_feedback").value
        )
        self.max_linear_x = abs(float(self.get_parameter("max_linear_x").value))
        self.max_linear_y = abs(float(self.get_parameter("max_linear_y").value))
        self.max_angular_z = abs(float(self.get_parameter("max_angular_z").value))
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.joint_state_timeout = float(
            self.get_parameter("joint_state_timeout").value
        )

        self.last_cmd: Twist | None = None
        self.last_cmd_time: float | None = None
        self.big_yaw: float | None = None
        self.big_yaw_time: float | None = None
        self.emergency_stop = False

        reliable = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.motion_pub = self.create_publisher(
            MotionCtrl, str(self.get_parameter("motion_control_topic").value), reliable
        )
        self.chassis_pub = self.create_publisher(
            Twist, str(self.get_parameter("chassis_topic").value), reliable
        )

        self.create_subscription(
            Twist,
            str(self.get_parameter("input_topic").value),
            self._cmd_callback,
            reliable,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._joint_state_callback,
            10,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("emergency_stop_topic").value),
            self._emergency_callback,
            reliable,
        )

        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            "gz_chassis_cmd_adapter: '%s' -> '%s' + '%s'"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("motion_control_topic").value,
                self.get_parameter("chassis_topic").value,
            )
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = msg
        self.last_cmd_time = self._now()

    def _joint_state_callback(self, msg: JointState) -> None:
        if self.big_yaw_joint_name in msg.name:
            index = msg.name.index(self.big_yaw_joint_name)
            if index < len(msg.position):
                self.big_yaw = float(msg.position[index])
                self.big_yaw_time = self._now()

    def _emergency_callback(self, msg: Bool) -> None:
        self.emergency_stop = bool(msg.data)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    def _resolve_command(self) -> tuple[float, float, float]:
        """Return the body-frame command, or an exact zero on any fault."""
        if self.emergency_stop:
            return 0.0, 0.0, 0.0
        if self.last_cmd is None or self.last_cmd_time is None:
            return 0.0, 0.0, 0.0
        if self.command_timeout > 0.0:
            if self._now() - self.last_cmd_time > self.command_timeout:
                return 0.0, 0.0, 0.0
        return (
            float(self.last_cmd.linear.x),
            float(self.last_cmd.linear.y),
            float(self.last_cmd.angular.z),
        )

    def _big_yaw_angle(self) -> float | None:
        if not self.transform_with_big_yaw:
            return 0.0
        if self.big_yaw is None or self.big_yaw_time is None:
            return None
        if self.joint_state_timeout > 0.0:
            if self._now() - self.big_yaw_time > self.joint_state_timeout:
                return None
        return self.big_yaw

    def _on_timer(self) -> None:
        vx, vy, wz = self._resolve_command()

        vx = self._clamp(vx, self.max_linear_x)
        vy = self._clamp(vy, self.max_linear_y)
        wz = self._clamp(wz, self.max_angular_z)

        motion = MotionCtrl()
        motion.linear_x = vx
        motion.linear_y = vy
        motion.angular_z = wz
        self.motion_pub.publish(motion)

        psi = self._big_yaw_angle()
        if psi is None:
            if self.require_big_yaw:
                nonzero = abs(vx) > 1e-9 or abs(vy) > 1e-9
                if nonzero:
                    self.get_logger().warn(
                        "Big-yaw feedback unavailable; publishing zero chassis velocity.",
                        throttle_duration_sec=2.0,
                    )
                chassis_vx, chassis_vy = 0.0, 0.0
                wz = 0.0 if nonzero else wz
            else:
                chassis_vx, chassis_vy = vx, vy
        else:
            cos_psi = math.cos(psi)
            sin_psi = math.sin(psi)
            chassis_vx = cos_psi * vx - sin_psi * vy
            chassis_vy = sin_psi * vx + cos_psi * vy

        twist = Twist()
        twist.linear.x = chassis_vx
        twist.linear.y = chassis_vy
        twist.angular.z = wz
        self.chassis_pub.publish(twist)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ChassisCmdAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
