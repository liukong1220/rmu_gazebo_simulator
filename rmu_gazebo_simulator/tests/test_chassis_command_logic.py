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

"""Regression tests for the Gazebo chassis command fallback contract."""

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


BRIDGE_DIR = Path(__file__).resolve().parents[1] / "scripts" / "ats_bridge"
sys.path.insert(0, str(BRIDGE_DIR))

from chassis_command_logic import transform_command


class ChassisCommandLogicTest(unittest.TestCase):
    def test_required_missing_big_yaw_zeros_both_outputs(self):
        output = transform_command(0.8, -0.4, 0.7, None, True, True)

        self.assertTrue(output.big_yaw_missing)
        self.assertEqual(
            (
                output.motion_vx,
                output.motion_vy,
                output.motion_wz,
                output.chassis_vx,
                output.chassis_vy,
                output.chassis_wz,
            ),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        )

    def test_available_big_yaw_rotates_only_chassis_linear_velocity(self):
        output = transform_command(1.0, 0.0, 0.4, math.pi / 2.0, True, True)

        self.assertFalse(output.big_yaw_missing)
        self.assertAlmostEqual(output.motion_vx, 1.0)
        self.assertAlmostEqual(output.motion_vy, 0.0)
        self.assertAlmostEqual(output.motion_wz, 0.4)
        self.assertAlmostEqual(output.chassis_vx, 0.0, places=12)
        self.assertAlmostEqual(output.chassis_vy, 1.0, places=12)
        self.assertAlmostEqual(output.chassis_wz, 0.4)


if __name__ == "__main__":
    unittest.main()
