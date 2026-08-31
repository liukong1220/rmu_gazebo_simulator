// Copyright 2026 ATS 2026 Sentry Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RMU_GAZEBO_SIMULATOR__TF_ESTABLISHMENT_TRACKER_HPP_
#define RMU_GAZEBO_SIMULATOR__TF_ESTABLISHMENT_TRACKER_HPP_

#include <cstddef>

namespace rmu_gazebo_simulator
{

// Separates two physically different TF outcomes that a single failure counter
// conflates.
//
// The recorder starts polling `map -> gimbal_yaw_odom` as soon as it spins,
// which is necessarily before the localization chain has published that
// transform: Point-LIO must converge and localization_fusion must emit its
// first `map -> odom` before any lookup can succeed. Every failure in that
// leading window is the expected absence of a transform, not a regression.
//
// Once the first lookup succeeds the chain is established, and from that point
// a failure means the previously resolvable chain no longer resolves. That is
// a genuine contract violation and is the quantity this P1 gate judges. This
// tracker does not inspect dynamic transform source stamps, so freshness is a
// separate admission field.
//
// The boundary is the first success, not a wall-clock timeout, so the
// classification never depends on how long this particular host took to warm
// up. A run that never establishes the chain reports zero post-establishment
// failures, so callers must also require `established()` before treating a
// zero count as healthy.
class TfEstablishmentTracker final
{
public:
  void observeSuccess()
  {
    ++attempts_;
    ++successes_;
    established_ = true;
  }

  void observeFailure()
  {
    ++attempts_;
    ++failures_;
    if (established_) {
      ++failures_after_establishment_;
    } else {
      ++failures_before_establishment_;
    }
  }

  bool established() const {return established_;}
  std::size_t attempts() const {return attempts_;}
  std::size_t successes() const {return successes_;}
  std::size_t failures() const {return failures_;}
  std::size_t failuresBeforeEstablishment() const {return failures_before_establishment_;}
  std::size_t failuresAfterEstablishment() const {return failures_after_establishment_;}

private:
  bool established_{false};
  std::size_t attempts_{0};
  std::size_t successes_{0};
  std::size_t failures_{0};
  std::size_t failures_before_establishment_{0};
  std::size_t failures_after_establishment_{0};
};

}  // namespace rmu_gazebo_simulator

#endif  // RMU_GAZEBO_SIMULATOR__TF_ESTABLISHMENT_TRACKER_HPP_
