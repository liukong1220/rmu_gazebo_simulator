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

#include "rmu_gazebo_simulator/tf_establishment_tracker.hpp"

int main()
{
  using rmu_gazebo_simulator::TfEstablishmentTracker;

  // Nothing observed yet: the chain is not established, so a zero
  // post-establishment count must not be readable as healthy on its own.
  {
    TfEstablishmentTracker tracker;
    assert(!tracker.established());
    assert(tracker.attempts() == 0);
    assert(tracker.failuresAfterEstablishment() == 0);
  }

  // The real observed baseline shape: a short leading burst of failures while
  // localization_fusion has not yet published map -> odom, then continuous
  // success. Total failures are nonzero, which is exactly what the old
  // single-counter gate rejected, but no established transform ever went
  // missing, so the run is healthy.
  {
    TfEstablishmentTracker tracker;
    tracker.observeFailure();
    tracker.observeFailure();
    for (int i = 0; i < 599; ++i) {
      tracker.observeSuccess();
    }
    assert(tracker.established());
    assert(tracker.attempts() == 601);
    assert(tracker.successes() == 599);
    assert(tracker.failures() == 2);
    assert(tracker.failuresBeforeEstablishment() == 2);
    assert(tracker.failuresAfterEstablishment() == 0);
  }

  // A transform that drops out after being established is a genuine violation
  // and must stay visible, even though the total failure count is identical to
  // the healthy case above.
  {
    TfEstablishmentTracker tracker;
    tracker.observeFailure();
    tracker.observeSuccess();
    tracker.observeFailure();
    tracker.observeFailure();
    tracker.observeSuccess();
    assert(tracker.established());
    assert(tracker.failures() == 3);
    assert(tracker.failuresBeforeEstablishment() == 1);
    assert(tracker.failuresAfterEstablishment() == 2);
  }

  // A chain that never comes up: every attempt failed. failuresAfterEstablishment
  // is zero here too, so established() is the field that separates this from a
  // healthy run.
  {
    TfEstablishmentTracker tracker;
    for (int i = 0; i < 600; ++i) {
      tracker.observeFailure();
    }
    assert(!tracker.established());
    assert(tracker.failures() == 600);
    assert(tracker.failuresBeforeEstablishment() == 600);
    assert(tracker.failuresAfterEstablishment() == 0);
  }

  // A perfectly clean run stays clean in every field.
  {
    TfEstablishmentTracker tracker;
    for (int i = 0; i < 10; ++i) {
      tracker.observeSuccess();
    }
    assert(tracker.established());
    assert(tracker.failures() == 0);
    assert(tracker.failuresBeforeEstablishment() == 0);
    assert(tracker.failuresAfterEstablishment() == 0);
  }

  return 0;
}
