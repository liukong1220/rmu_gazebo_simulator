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

// A one-shot, read-only health witness for scripts/test_gazebo_minco_mpc_chain.sh.
// It must not publish a control, map, TF, or status topic.  Wall time is
// intentional: a paused/reset simulation clock must not make a test deadline
// appear to pass or expire spuriously.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

#include <ats_navigation_interfaces/msg/localization_status.hpp>
#include <ats_navigation_interfaces/msg/planning_map_status.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

class NavigationHealthProbe final : public rclcpp::Node
{
public:
  NavigationHealthProbe()
  : Node(
      "ats_navigation_health_probe",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", false))
  {
    const auto localization_topic = declare_parameter<std::string>(
      "localization_status_topic", "/localization/status");
    const auto map_ready_topic = declare_parameter<std::string>(
      "map_ready_topic", "/rog_map_adapter/ready");
    const auto map_status_topic = declare_parameter<std::string>(
      "map_status_topic", "/rog_map_adapter/status");
    const auto planning_grid_topic = declare_parameter<std::string>(
      "planning_grid_topic", "/rc_esdf/planning_grid");

    required_stable_samples_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<std::int64_t>("required_stable_samples", 3)));
    timeout_sec_ = std::max(0.1, declare_parameter<double>("timeout_sec", 120.0));
    sample_period_sec_ = std::max(0.05, declare_parameter<double>("sample_period_sec", 1.0));
    localization_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("localization_timeout_sec", 1.0));
    map_timeout_sec_ = std::max(0.1, declare_parameter<double>("map_timeout_sec", 5.0));

    const auto health_qos = rclcpp::QoS(1).reliable().transient_local();
    localization_sub_ = create_subscription<
      ats_navigation_interfaces::msg::LocalizationStatus>(
      localization_topic, health_qos,
      [this](const ats_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr message) {
        localization_ = *message;
        localization_received_ = SteadyClock::now();
      });
    map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      map_ready_topic, health_qos,
      [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
        map_ready_ = message->data;
        map_ready_received_ = SteadyClock::now();
      });
    map_status_sub_ = create_subscription<
      ats_navigation_interfaces::msg::PlanningMapStatus>(
      map_status_topic, health_qos,
      [this](const ats_navigation_interfaces::msg::PlanningMapStatus::ConstSharedPtr message) {
        map_status_ = *message;
        map_status_received_ = SteadyClock::now();
      });
    planning_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      planning_grid_topic, health_qos,
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message) {
        const auto expected_cells = static_cast<std::size_t>(message->info.width) *
          static_cast<std::size_t>(message->info.height);
        planning_grid_payload_ = expected_cells > 0 && message->data.size() == expected_cells;
        planning_grid_received_ = SteadyClock::now();
      });

    started_ = SteadyClock::now();
    sample_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(sample_period_sec_)),
      std::bind(&NavigationHealthProbe::evaluate, this));
  }

  bool succeeded() const { return succeeded_; }

private:
  static double ageSeconds(const std::optional<SteadyTime> & received, const SteadyTime & now)
  {
    if (!received) {
      return -1.0;
    }
    return std::chrono::duration<double>(now - *received).count();
  }

  bool healthy(const SteadyTime & now) const
  {
    if (!localization_ || !map_ready_ || !map_status_ || !planning_grid_received_) {
      return false;
    }
    const auto localization_age = ageSeconds(localization_received_, now);
    const auto map_ready_age = ageSeconds(map_ready_received_, now);
    const auto map_status_age = ageSeconds(map_status_received_, now);
    const auto grid_age = ageSeconds(planning_grid_received_, now);
    return localization_->state ==
             ats_navigation_interfaces::msg::LocalizationStatus::STATE_TRACKING &&
           localization_age >= 0.0 && localization_age <= localization_timeout_sec_ &&
           *map_ready_ && map_ready_age >= 0.0 && map_ready_age <= map_timeout_sec_ &&
           map_status_->ready && map_status_age >= 0.0 && map_status_age <= map_timeout_sec_ &&
           planning_grid_payload_ && grid_age >= 0.0 && grid_age <= map_timeout_sec_ &&
           map_status_->localization_epoch == localization_->epoch;
  }

  void evaluate()
  {
    if (finished_) {
      return;
    }
    const auto now = SteadyClock::now();
    if (healthy(now)) {
      ++stable_samples_;
      if (stable_samples_ >= required_stable_samples_) {
        finish(true, now);
        return;
      }
    } else {
      stable_samples_ = 0;
    }
    if (std::chrono::duration<double>(now - started_).count() >= timeout_sec_) {
      finish(false, now);
    }
  }

  void finish(bool succeeded, const SteadyTime & now)
  {
    succeeded_ = succeeded;
    finished_ = true;
    sample_timer_->cancel();
    const auto localization_age = ageSeconds(localization_received_, now);
    const auto map_ready_age = ageSeconds(map_ready_received_, now);
    const auto map_status_age = ageSeconds(map_status_received_, now);
    const auto grid_age = ageSeconds(planning_grid_received_, now);
    const int localization_state = localization_ ? static_cast<int>(localization_->state) : -1;
    const std::uint64_t localization_epoch = localization_ ? localization_->epoch : 0U;
    const bool map_ready = map_ready_.value_or(false);
    const bool map_status_ready = map_status_ && map_status_->ready;
    const std::uint64_t map_epoch = map_status_ ? map_status_->localization_epoch : 0U;
    const std::uint64_t source_generation = map_status_ ? map_status_->rog_generation : 0U;
    const std::uint64_t publication_sequence = map_status_ ? map_status_->publication_sequence : 0U;
    std::cout
      << "ATS_HEALTH_PROBE_RESULT ready=" << (succeeded ? "yes" : "no")
      << " stable=" << stable_samples_ << "/" << required_stable_samples_
      << " localization_state=" << localization_state
      << " localization_epoch=" << localization_epoch
      << " localization_age_s=" << localization_age
      << " map_ready=" << (map_ready ? "true" : "false")
      << " map_ready_age_s=" << map_ready_age
      << " map_status_ready=" << (map_status_ready ? "true" : "false")
      << " map_status_epoch=" << map_epoch
      << " map_status_age_s=" << map_status_age
      << " grid_payload=" << (planning_grid_payload_ ? "yes" : "no")
      << " grid_age_s=" << grid_age
      << " source_generation=" << source_generation
      << " publication_sequence=" << publication_sequence
      << std::endl;
    rclcpp::shutdown();
  }

  int required_stable_samples_{3};
  double timeout_sec_{120.0};
  double sample_period_sec_{1.0};
  double localization_timeout_sec_{1.0};
  double map_timeout_sec_{5.0};
  SteadyTime started_;
  bool finished_{false};
  bool succeeded_{false};
  int stable_samples_{0};
  bool planning_grid_payload_{false};
  std::optional<ats_navigation_interfaces::msg::LocalizationStatus> localization_;
  std::optional<bool> map_ready_;
  std::optional<ats_navigation_interfaces::msg::PlanningMapStatus> map_status_;
  std::optional<SteadyTime> localization_received_;
  std::optional<SteadyTime> map_ready_received_;
  std::optional<SteadyTime> map_status_received_;
  std::optional<SteadyTime> planning_grid_received_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlanningMapStatus>::SharedPtr
    map_status_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_grid_sub_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<NavigationHealthProbe>();
  rclcpp::spin(node);
  return node->succeeded() ? 0 : 1;
}
