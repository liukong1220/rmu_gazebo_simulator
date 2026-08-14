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

#ifndef RMU_GAZEBO_SIMULATOR__SWERVE_KINEMATICS_HPP_
#define RMU_GAZEBO_SIMULATOR__SWERVE_KINEMATICS_HPP_

#include <cmath>
#include <stdexcept>

namespace rmu_gazebo_simulator::swerve
{

inline constexpr double kPi = 3.14159265358979323846;

// This matches the HERO controller and ATS MPC contract: x is forward, y is
// left and the command is expressed in the chassis frame.
struct BodyTwist
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct ModulePosition
{
  double x{0.0};
  double y{0.0};
};

struct ModuleSetpoint
{
  double steering_angle{0.0};
  double wheel_angular_velocity{0.0};
};

inline double WrapPi(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

// The wheel centre velocity is v_i = [vx - wz * y_i, vy + wz * x_i].
// For an idle module, retain the previous target to avoid commanding an
// arbitrary steering motion while the wheel velocity is exactly zero.
inline ModuleSetpoint ComputeModuleSetpoint(
  const BodyTwist & command,
  const ModulePosition & position,
  const double wheel_radius,
  const double measured_steering_angle,
  const double previous_target_angle)
{
  if (!(wheel_radius > 0.0)) {
    throw std::invalid_argument("wheel_radius must be positive");
  }

  const double module_vx = command.vx - command.wz * position.y;
  const double module_vy = command.vy + command.wz * position.x;
  const double module_speed = std::hypot(module_vx, module_vy);
  if (module_speed <= 1e-9) {
    return {previous_target_angle, 0.0};
  }

  ModuleSetpoint setpoint{
    std::atan2(module_vy, module_vx), module_speed / wheel_radius};
  if (std::abs(WrapPi(setpoint.steering_angle - measured_steering_angle)) > kPi / 2.0) {
    setpoint.steering_angle = WrapPi(setpoint.steering_angle + kPi);
    setpoint.wheel_angular_velocity = -setpoint.wheel_angular_velocity;
  }
  return setpoint;
}

}  // namespace rmu_gazebo_simulator::swerve

#endif  // RMU_GAZEBO_SIMULATOR__SWERVE_KINEMATICS_HPP_
