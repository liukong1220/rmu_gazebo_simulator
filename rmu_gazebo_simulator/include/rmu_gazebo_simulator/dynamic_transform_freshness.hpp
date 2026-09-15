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

#ifndef RMU_GAZEBO_SIMULATOR__DYNAMIC_TRANSFORM_FRESHNESS_HPP_
#define RMU_GAZEBO_SIMULATOR__DYNAMIC_TRANSFORM_FRESHNESS_HPP_

#include <cstdint>
#include <optional>
#include <vector>

#include "rmu_gazebo_simulator/evidence_statistics.hpp"

namespace rmu_gazebo_simulator
{

// Freshness witness for the dynamic edge of a polled TF lookup.
//
// TfEstablishmentTracker separates "the chain never came up" from "the chain
// came up and broke". Neither can see a broadcaster that died while the TF
// buffer keeps replaying its last transform: a TimePointZero lookup keeps
// succeeding forever, so attempts/successes/failures all read healthy. This
// witness closes that hole by judging the source stamp carried by the
// transform the lookup actually returned (transform.header.stamp), which is
// why observeTf() must read that field instead of discarding it.
//
// Samples come from two clock domains only:
//   * the source stamp of the returned transform (written by the
//     broadcaster), one per successful lookup, and
//   * the latest /clock stamp at the moment of the poll (the consumer's
//     reference clock).
//
// Derived evidence, with the semantics the P1 admission gates read:
//   tf_dynamic_samples                successful lookups observed
//   tf_dynamic_distinct_stamp_updates stamps that advanced since the
//                                     previous poll, plus the first readable
//                                     stamp (the chain started)
//   tf_dynamic_duplicate_stamps       polls whose stamp did not advance
//   tf_dynamic_backward_stamps        stamps that went backwards
//   tf_dynamic_invalid_stamps         lookups whose stamp was absent/invalid
//   tf_dynamic_age_*                  /clock minus the returned source stamp
//                                     (absolute consumer-side lag); sampled
//                                     on every poll where /clock is known
//   tf_dynamic_stamp_staleness_*      /clock elapsed since the stamp last
//                                     advanced (freeze evidence); sampled on
//                                     every poll once an advance exists
//   tf_dynamic_update_gap_*           source-stamp delta between consecutive
//                                     advances (stall evidence in the
//                                     producer's clock domain, immune to a
//                                     consumer-side /clock catch-up)
//   tf_dynamic_age_floor_s            minimum observed age, diagnosis only:
//                                     gated percentiles are never diffed
//                                     against an estimated floor (domain 137)
//   tf_dynamic_backward_clock_samples reference-clock regressions that would
//                                     distort the staleness samples taken
//                                     around them
//
// Age and staleness need the reference clock, so a lookup that arrives before
// any /clock message contributes cadence evidence but no age sample. Staleness
// needs at least one recorded advance, so a never-advancing chain contributes
// no staleness sample - the runner's staleness-samples gate fails closed on
// that shape instead of gating on a percentile of nothing.
class DynamicTransformFreshness final
{
public:
  void observeClock(const std::int64_t clock_stamp_ns)
  {
    if (previous_clock_stamp_ns_ && clock_stamp_ns < *previous_clock_stamp_ns_) {
      ++backward_clock_samples_;
    }
    previous_clock_stamp_ns_ = clock_stamp_ns;
    latest_clock_stamp_ns_ = clock_stamp_ns;
  }

  // Record one successful lookup whose returned transform carried source
  // stamp `transform_stamp_ns`. Pass valid_stamp=false when the lookup
  // succeeded but the returned stamp could not be read; that is an evidence
  // defect and is counted, never silently skipped.
  void observeLookup(const std::int64_t transform_stamp_ns, const bool valid_stamp = true)
  {
    ++samples_;
    if (!valid_stamp || transform_stamp_ns <= 0) {
      ++invalid_stamps_;
      return;
    }

    bool advanced = false;
    if (previous_stamp_ns_) {
      if (transform_stamp_ns == *previous_stamp_ns_) {
        ++duplicate_stamps_;
      } else if (transform_stamp_ns < *previous_stamp_ns_) {
        ++backward_stamps_;
      } else {
        advanced = true;
        ++distinct_stamp_updates_;
        if (previous_update_stamp_ns_) {
          update_gaps_sec_.push_back(
            static_cast<double>(transform_stamp_ns - *previous_update_stamp_ns_) *
            1e-9);
        }
        previous_update_stamp_ns_ = transform_stamp_ns;
      }
    } else {
      // First readable stamp: an update by definition (the chain started),
      // with no earlier stamp to measure a gap against.
      advanced = true;
      ++distinct_stamp_updates_;
      previous_update_stamp_ns_ = transform_stamp_ns;
    }
    previous_stamp_ns_ = transform_stamp_ns;

    if (latest_clock_stamp_ns_ && *latest_clock_stamp_ns_ > 0) {
      const double age =
        static_cast<double>(*latest_clock_stamp_ns_ - transform_stamp_ns) * 1e-9;
      ages_sec_.push_back(age);
      if (age < 0.0) {
        ++future_stamps_;
      }
      if (!age_floor_sec_ || age < *age_floor_sec_) {
        age_floor_sec_ = age;
      }
      // Staleness is "how much /clock elapsed since the stamp last
      // advanced", measured against the most recent advance strictly before
      // this poll. Sampling the advance's own poll would always read ~0,
      // and diffing stamp values instead would fold a lag shared by every
      // sample back in - domain 139 read a healthy 0.200 s staleness p99 on
      // an edge 2.06 s behind precisely because the shared lag cancels here.
      if (last_advance_clock_ns_) {
        staleness_sec_.push_back(
          static_cast<double>(*latest_clock_stamp_ns_ - *last_advance_clock_ns_) *
          1e-9);
      }
    }
    if (advanced && latest_clock_stamp_ns_ && *latest_clock_stamp_ns_ > 0) {
      last_advance_clock_ns_ = latest_clock_stamp_ns_;
    }
  }

  std::size_t samples() const {return samples_;}
  std::size_t distinctStampUpdates() const {return distinct_stamp_updates_;}
  std::size_t duplicateStamps() const {return duplicate_stamps_;}
  std::size_t backwardStamps() const {return backward_stamps_;}
  std::size_t invalidStamps() const {return invalid_stamps_;}
  std::size_t futureStamps() const {return future_stamps_;}
  std::size_t backwardClockSamples() const {return backward_clock_samples_;}
  const std::vector<double> & agesSec() const {return ages_sec_;}
  const std::vector<double> & stalenessSec() const {return staleness_sec_;}
  const std::vector<double> & updateGapsSec() const {return update_gaps_sec_;}
  std::optional<double> ageFloorSec() const {return age_floor_sec_;}

  double agePercentile(const double probability) const
  {
    return percentile(ages_sec_, probability);
  }

  double stalenessPercentile(const double probability) const
  {
    return percentile(staleness_sec_, probability);
  }

  double updateGapPercentile(const double probability) const
  {
    return percentile(update_gaps_sec_, probability);
  }

  std::size_t ageSamples() const {return ages_sec_.size();}
  std::size_t stalenessSamples() const {return staleness_sec_.size();}
  std::size_t updateGapSamples() const {return update_gaps_sec_.size();}

private:
  std::size_t samples_{0};
  std::size_t distinct_stamp_updates_{0};
  std::size_t duplicate_stamps_{0};
  std::size_t backward_stamps_{0};
  std::size_t invalid_stamps_{0};
  std::size_t future_stamps_{0};
  std::size_t backward_clock_samples_{0};
  std::optional<std::int64_t> previous_stamp_ns_;
  std::optional<std::int64_t> previous_update_stamp_ns_;
  std::optional<std::int64_t> last_advance_clock_ns_;
  std::optional<std::int64_t> latest_clock_stamp_ns_;
  std::optional<std::int64_t> previous_clock_stamp_ns_;
  std::vector<double> ages_sec_;
  std::vector<double> staleness_sec_;
  std::vector<double> update_gaps_sec_;
  std::optional<double> age_floor_sec_;
};

}  // namespace rmu_gazebo_simulator

#endif  // RMU_GAZEBO_SIMULATOR__DYNAMIC_TRANSFORM_FRESHNESS_HPP_
