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

"""Pure command transformation for the Gazebo chassis adapter."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ChassisCommandOutputs:
    """Body-frame MotionCtrl and chassis-frame Twist values from one input."""

    motion_vx: float
    motion_vy: float
    motion_wz: float
    chassis_vx: float
    chassis_vy: float
    chassis_wz: float
    big_yaw_missing: bool = False


def transform_command(
    vx: float,
    vy: float,
    wz: float,
    big_yaw: float | None,
    transform_with_big_yaw: bool,
    require_big_yaw: bool,
) -> ChassisCommandOutputs:
    """Return both actuator outputs, failing closed for a required yaw sample."""
    if transform_with_big_yaw and big_yaw is None:
        if require_big_yaw:
            return ChassisCommandOutputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, True)
        return ChassisCommandOutputs(vx, vy, wz, vx, vy, wz, True)

    psi = big_yaw if transform_with_big_yaw else 0.0
    cos_psi = math.cos(psi)
    sin_psi = math.sin(psi)
    return ChassisCommandOutputs(
        vx,
        vy,
        wz,
        cos_psi * vx - sin_psi * vy,
        sin_psi * vx + cos_psi * vy,
        wz,
    )
