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

#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

namespace rmu_gazebo_simulator
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

// Bounded-run timing witness.  Receipt timing intentionally uses steady time;
// message stamps remain separate so a paused or reset simulation is visible.
struct EvidenceStatistics
{
  std::size_t samples{0};
  std::optional<SteadyTime> previous_receipt;
  std::optional<std::int64_t> previous_stamp_ns;
  std::vector<double> wall_intervals_sec;
  std::vector<double> stamp_intervals_sec;
  std::vector<double> stamp_ages_sec;
  std::size_t duplicate_stamp_count{0};
  std::size_t backward_stamp_count{0};
  std::size_t invalid_stamp_count{0};
  std::size_t future_stamp_count{0};

  void observeReceipt(const SteadyTime receipt = SteadyClock::now())
  {
    if (previous_receipt) {
      wall_intervals_sec.push_back(
        std::chrono::duration<double>(receipt - *previous_receipt).count());
    }
    previous_receipt = receipt;
    ++samples;
  }

  void observeStamp(const std::int64_t stamp_ns)
  {
    if (stamp_ns <= 0) {
      ++invalid_stamp_count;
      return;
    }
    if (previous_stamp_ns) {
      const auto delta_ns = stamp_ns - *previous_stamp_ns;
      if (delta_ns == 0) {
        ++duplicate_stamp_count;
      } else if (delta_ns < 0) {
        ++backward_stamp_count;
      } else {
        stamp_intervals_sec.push_back(static_cast<double>(delta_ns) * 1e-9);
      }
    }
    previous_stamp_ns = stamp_ns;
  }

  void observeAge(const std::optional<std::int64_t> & reference_stamp_ns,
    const std::int64_t message_stamp_ns)
  {
    if (!reference_stamp_ns || *reference_stamp_ns <= 0 || message_stamp_ns <= 0) {
      return;
    }
    const double age = static_cast<double>(*reference_stamp_ns - message_stamp_ns) * 1e-9;
    stamp_ages_sec.push_back(age);
    if (age < 0.0) {
      ++future_stamp_count;
    }
  }
};

inline double percentile(const std::vector<double> & values, const double probability)
{
  if (values.empty()) {
    return 0.0;
  }
  auto sorted = values;
  std::sort(sorted.begin(), sorted.end());
  const auto rank = static_cast<std::size_t>(std::ceil(
    std::clamp(probability, 0.0, 1.0) * static_cast<double>(sorted.size())));
  return sorted[std::min(sorted.size() - 1U, rank == 0U ? 0U : rank - 1U)];
}

}  // namespace rmu_gazebo_simulator
