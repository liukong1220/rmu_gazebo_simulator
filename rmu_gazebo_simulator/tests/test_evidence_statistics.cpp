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

#include <cassert>
#include <chrono>
#include <cmath>

#include "rmu_gazebo_simulator/evidence_statistics.hpp"

int main()
{
  using rmu_gazebo_simulator::EvidenceStatistics;
  using rmu_gazebo_simulator::SteadyTime;

  EvidenceStatistics statistics;
  const SteadyTime start{};
  statistics.observeReceipt(start);
  statistics.observeStamp(1'000'000'000);
  statistics.observeAge(1'500'000'000, 1'000'000'000);
  statistics.observeReceipt(start + std::chrono::milliseconds(100));
  statistics.observeStamp(1'200'000'000);
  statistics.observeAge(1'000'000'000, 1'200'000'000);
  statistics.observeReceipt(start + std::chrono::milliseconds(150));
  statistics.observeStamp(1'200'000'000);
  statistics.observeReceipt(start + std::chrono::milliseconds(200));
  statistics.observeStamp(1'100'000'000);
  statistics.observePublicationSequence(20U);
  statistics.observePublicationSequence(21U);
  statistics.observePublicationSequence(24U);
  statistics.observePublicationSequence(23U);
  statistics.observeCallbackDuration(
    start + std::chrono::milliseconds(200),
    start + std::chrono::milliseconds(203));
  statistics.observeCallbackDuration(
    start + std::chrono::milliseconds(204),
    start + std::chrono::milliseconds(206));

  assert(statistics.samples == 4U);
  // Every arrival after the first contributes one wall interval, even when
  // its ROS stamp is duplicate or moves backward. Arrival cadence and stamp
  // ordering are independent diagnostic dimensions.
  assert(statistics.wall_intervals_sec.size() == 3U);
  assert(std::abs(statistics.wall_intervals_sec[0] - 0.1) < 1e-12);
  assert(std::abs(statistics.wall_intervals_sec[1] - 0.05) < 1e-12);
  assert(std::abs(statistics.wall_intervals_sec[2] - 0.05) < 1e-12);
  assert(statistics.stamp_intervals_sec.size() == 1U);
  assert(statistics.duplicate_stamp_count == 1U);
  assert(statistics.backward_stamp_count == 1U);
  assert(statistics.future_stamp_count == 1U);
  assert(statistics.publication_sequence_samples == 4U);
  assert(statistics.publication_sequence_gap_count == 1U);
  assert(statistics.publication_sequence_missing_count == 2U);
  assert(statistics.publication_sequence_nonmonotonic_count == 1U);
  assert(statistics.callback_durations_sec.size() == 2U);
  assert(rmu_gazebo_simulator::percentile(statistics.wall_intervals_sec, 0.99) > 0.09);
  assert(rmu_gazebo_simulator::percentile(statistics.stamp_ages_sec, 0.50) < 0.0);
  assert(std::abs(rmu_gazebo_simulator::percentile(statistics.stamp_ages_sec, 0.95) - 0.5) < 1e-12);
  assert(std::abs(
    rmu_gazebo_simulator::percentile(statistics.callback_durations_sec, 0.95) - 0.003) < 1e-12);
  return 0;
}
