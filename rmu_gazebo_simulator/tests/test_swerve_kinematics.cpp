// Copyright 2026 ATS 2026 Sentry Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "rmu_gazebo_simulator/swerve_kinematics.hpp"

namespace
{
using rmu_gazebo_simulator::swerve::BodyTwist;
using rmu_gazebo_simulator::swerve::ComputeModuleSetpoint;
using rmu_gazebo_simulator::swerve::kPi;
using rmu_gazebo_simulator::swerve::ModulePosition;
using rmu_gazebo_simulator::swerve::ModuleSetpoint;
using rmu_gazebo_simulator::swerve::WrapPi;

constexpr double kWheelRadius = 0.0425;
constexpr double kTolerance = 1e-9;

bool ExpectNear(const std::string & label, const double actual, const double expected)
{
  if (std::abs(actual - expected) <= kTolerance) {
    return true;
  }
  std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
  return false;
}

bool ExpectAngle(const std::string & label, const double actual, const double expected)
{
  return ExpectNear(label, WrapPi(actual - expected), 0.0);
}

ModuleSetpoint Setpoint(
  const BodyTwist & command, const ModulePosition & position,
  const double measured_angle = 0.0, const double previous_target = 0.0)
{
  return ComputeModuleSetpoint(
    command, position, kWheelRadius, measured_angle, previous_target);
}

bool TestForwardAndLateralTranslation()
{
  const std::array<ModulePosition, 4> positions{{
      {0.270, 0.270}, {0.270, -0.270}, {-0.270, 0.270}, {-0.270, -0.270}}};
  bool passed = true;
  for (const auto & position : positions) {
    const auto forward = Setpoint({1.0, 0.0, 0.0}, position);
    passed &= ExpectAngle("forward steering", forward.steering_angle, 0.0);
    passed &= ExpectNear("forward wheel speed", forward.wheel_angular_velocity, 1.0 / kWheelRadius);

    const auto lateral = Setpoint({0.0, 1.0, 0.0}, position);
    passed &= ExpectAngle("left steering", lateral.steering_angle, kPi / 2.0);
    passed &= ExpectNear("left wheel speed", lateral.wheel_angular_velocity, 1.0 / kWheelRadius);
  }
  return passed;
}

bool TestCounterClockwiseRotation()
{
  const std::array<ModulePosition, 4> positions{{
      {0.270, 0.270}, {0.270, -0.270}, {-0.270, 0.270}, {-0.270, -0.270}}};
  bool passed = true;
  for (const auto & position : positions) {
    const auto setpoint = Setpoint({0.0, 0.0, 1.0}, position);
    const double reconstructed_speed = setpoint.wheel_angular_velocity * kWheelRadius;
    passed &= ExpectNear(
      "ccw module vx", reconstructed_speed * std::cos(setpoint.steering_angle), -position.y);
    passed &= ExpectNear(
      "ccw module vy", reconstructed_speed * std::sin(setpoint.steering_angle), position.x);
  }
  return passed;
}

bool TestMixedCommandAndShortestSteering()
{
  bool passed = true;
  const ModulePosition position{0.270, -0.270};
  const auto mixed = Setpoint({0.60, -0.30, 0.80}, position);
  const double expected_vx = 0.60 - 0.80 * position.y;
  const double expected_vy = -0.30 + 0.80 * position.x;
  passed &= ExpectAngle("mixed steering", mixed.steering_angle, std::atan2(expected_vy, expected_vx));
  passed &= ExpectNear(
    "mixed wheel speed", mixed.wheel_angular_velocity,
    std::hypot(expected_vx, expected_vy) / kWheelRadius);

  const auto reversed = Setpoint({-1.0, 0.0, 0.0}, ModulePosition{}, 0.0);
  passed &= ExpectAngle("shortest steering", reversed.steering_angle, 0.0);
  passed &= ExpectNear("shortest steering wheel reversal", reversed.wheel_angular_velocity, -1.0 / kWheelRadius);
  return passed;
}

bool TestIdleModuleHoldsPreviousSteeringTarget()
{
  const auto idle = Setpoint({0.0, 0.0, 0.0}, ModulePosition{}, 0.60, -0.45);
  return ExpectAngle("idle steering target", idle.steering_angle, -0.45) &&
         ExpectNear("idle wheel speed", idle.wheel_angular_velocity, 0.0);
}

}  // namespace

int main()
{
  const bool passed =
    TestForwardAndLateralTranslation() &&
    TestCounterClockwiseRotation() &&
    TestMixedCommandAndShortestSteering() &&
    TestIdleModuleHoldsPreviousSteeringTarget();
  return passed ? EXIT_SUCCESS : EXIT_FAILURE;
}
