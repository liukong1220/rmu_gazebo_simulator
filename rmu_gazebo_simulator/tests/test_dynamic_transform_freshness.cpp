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
#include <cmath>
#include <cstdio>
#include <vector>

#include "rmu_gazebo_simulator/dynamic_transform_freshness.hpp"

using rmu_gazebo_simulator::DynamicTransformFreshness;

namespace
{

constexpr std::int64_t kSec = 1000000000LL;

// 10 Hz broadcaster, 10 Hz /clock, polled at 10 Hz: the healthy shape.
std::vector<std::int64_t> healthyStamps()
{
  std::vector<std::int64_t> stamps;
  for (int i = 1; i <= 100; ++i) {
    stamps.push_back(i * kSec / 10);
  }
  return stamps;
}

}  // namespace

int main()
{
// Healthy chain: updates advance one period behind the clock, so ages are
// ~0.1 s, staleness ~0.1 s, and the update gaps ~0.1 s.
{
  DynamicTransformFreshness witness;
  for (int i = 0; i < 100; ++i) {
    witness.observeClock((i + 2) * kSec / 10);
    witness.observeLookup((i + 1) * kSec / 10);
  }
  assert(witness.samples() == 100);
  assert(witness.distinctStampUpdates() == 100);
  assert(witness.duplicateStamps() == 0);
  assert(witness.backwardStamps() == 0);
  assert(witness.ageSamples() == 100);
  assert(witness.stalenessSamples() == 99);
  assert(witness.updateGapSamples() == 99);
  // Age is one broadcast period behind the reference clock.
  assert(std::abs(witness.agePercentile(0.99) - 0.1) < 0.05);
  // No sample exceeds a 0.5 s age/staleness/gap limit.
  assert(witness.agePercentile(1.0) < 0.5);
  assert(witness.stalenessPercentile(1.0) < 0.5);
  assert(witness.updateGapPercentile(1.0) < 0.5);
  std::printf("healthy: ok\n");
}

  // Frozen broadcaster after the first stamp: the TimePointZero lookup keeps
  // succeeding, so only the stamp evidence reveals it. This is the exact
  // hole the witness exists to close.
  {
    DynamicTransformFreshness witness;
    witness.observeClock(1 * kSec);
    witness.observeLookup(1 * kSec);
    for (int i = 2; i <= 60; ++i) {
      witness.observeClock(i * kSec);
      witness.observeLookup(1 * kSec);  // frozen stamp, replayed by the buffer
    }
    assert(witness.samples() == 60);
    assert(witness.distinctStampUpdates() == 1);
    assert(witness.duplicateStamps() == 59);
    // The 1-update count is what the min-rate gate rejects; the ages and
    // staleness also grow past every limit.
    assert(witness.agePercentile(1.0) > 50.0);
    assert(witness.stalenessPercentile(1.0) > 50.0);
    std::printf("frozen: ok\n");
  }

  // Sustained lag: every stamp 2.4 s behind a live clock. Cadence stays
  // healthy, staleness ~0.1 s cancels the shared lag; only the age sees it.
  {
    DynamicTransformFreshness witness;
    const std::int64_t lag = 24 * kSec / 10;
    for (int i = 1; i <= 100; ++i) {
      witness.observeClock((i + 25) * kSec / 10);
      witness.observeLookup((i + 25) * kSec / 10 - lag);
    }
    assert(witness.distinctStampUpdates() == 100);
    assert(witness.updateGapPercentile(1.0) < 0.5);
    assert(witness.stalenessPercentile(1.0) < 0.5);
    assert(witness.agePercentile(0.99) > 2.0);
    std::printf("sustained_lag: ok\n");
  }

  // Mid-run stall: the stamp freezes for 1.5 s (15 polls at 10 Hz). Update
  // count and the p99 gates stay healthy; the update-gap max and the age max
  // both exceed their limits.
  {
    DynamicTransformFreshness witness;
    constexpr std::int64_t start = 10 * kSec;
    for (int i = 0; i < 60; ++i) {
      witness.observeClock(start + (i + 1) * kSec / 10);
      std::int64_t stamp = start + i * kSec / 10;
      if (i >= 30 && i < 45) {
        stamp = start + 30 * kSec / 10;  // frozen: 15 polls without advance
      }
      witness.observeLookup(stamp);
    }
    assert(witness.updateGapPercentile(1.0) >= 1.4);
    assert(witness.agePercentile(1.0) >= 1.4);
    assert(witness.stalenessPercentile(1.0) >= 1.4);
    std::printf("mid_run_stall: ok\n");
  }

  // Backward stamps and invalid stamps are counted, never silently dropped.
  {
    DynamicTransformFreshness witness;
    witness.observeClock(1 * kSec);
    witness.observeLookup(1 * kSec);
    witness.observeClock(2 * kSec);
    witness.observeLookup(1 * kSec - 5);  // backward
    witness.observeClock(3 * kSec);
    witness.observeLookup(0);  // invalid
    assert(witness.backwardStamps() == 1);
    assert(witness.invalidStamps() == 1);
    std::printf("stamp_anomalies: ok\n");
  }

  // Future-stamped chain (localization_fusion +0.05 s offset): negative ages
  // are legitimate, so they must be recorded, not rejected.
  {
    DynamicTransformFreshness witness;
    for (int i = 1; i <= 100; ++i) {
      witness.observeClock(i * kSec / 10);
      witness.observeLookup(i * kSec / 10 + kSec / 20);
    }
    assert(witness.samples() == 100);
    assert(witness.distinctStampUpdates() == 100);
    assert(witness.futureStamps() == 100);
    assert(witness.agePercentile(0.99) < 0.0);
    std::printf("future_stamps: ok\n");
  }

  // No /clock before the first lookups: cadence is still measured, age is
  // not, so the age sample count stays honest instead of borrowing zeros.
  {
    DynamicTransformFreshness witness;
    witness.observeLookup(1 * kSec);
    witness.observeLookup(2 * kSec);
    witness.observeClock(25 * kSec / 100);
    witness.observeLookup(3 * kSec);
    assert(witness.samples() == 3);
    assert(witness.distinctStampUpdates() == 3);
    assert(witness.ageSamples() == 1);
    assert(witness.ageFloorSec().has_value());
    std::printf("clock_gating: ok\n");
  }

  // A reference clock that steps backwards is counted separately: those
  // samples would corrupt the staleness distribution they were taken in.
  {
    DynamicTransformFreshness witness;
    witness.observeClock(2 * kSec);
    witness.observeClock(1 * kSec);
    assert(witness.backwardClockSamples() == 1);
    std::printf("backward_clock: ok\n");
  }

  std::printf("dynamic transform freshness witness: all checks passed\n");
  return 0;
}
