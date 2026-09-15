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

// A bounded, read-only witness for the Gazebo navigation regression.  It
// replaces a set of long-running `ros2 topic echo` processes, which can itself
// consume enough CPU to perturb the Point-LIO -> planning timing being tested.
// It intentionally uses a wall-clock deadline: a paused or reset simulation
// clock must not make evidence collection hang or end immediately.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ats_navigation_interfaces/msg/planning_map_status.hpp>
#include <ats_navigation_interfaces/msg/localization_status.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/transport/Node.hh>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rmw/features.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rmu_gazebo_simulator/dynamic_transform_freshness.hpp"
#include "rmu_gazebo_simulator/evidence_statistics.hpp"
#include "rmu_gazebo_simulator/tf_establishment_tracker.hpp"

namespace
{

using rmu_gazebo_simulator::DynamicTransformFreshness;
using rmu_gazebo_simulator::EvidenceStatistics;
using rmu_gazebo_simulator::SteadyClock;
using rmu_gazebo_simulator::SteadyTime;
using rmu_gazebo_simulator::TfEstablishmentTracker;

class NavigationEvidenceRecorder final : public rclcpp::Node
{
public:
  NavigationEvidenceRecorder()
  : Node(
      "ats_navigation_evidence_recorder",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", false))
  {
    robot_name_ = declare_parameter<std::string>(
      "robot_name", "red_standard_robot1");
    duration_sec_ = std::max(0.1, declare_parameter<double>("duration_sec", 30.0));
    exit_on_nonzero_command_ = declare_parameter<bool>("exit_on_nonzero_command", false);
    observe_gazebo_transport_lidar_ = declare_parameter<bool>(
      "observe_gazebo_transport_lidar", false);
    gazebo_lidar_transport_topic_ = declare_parameter<std::string>(
      "gazebo_lidar_transport_topic",
      "/world/default/model/" + robot_name_ +
      "/link/front_mid360/sensor/front_mid360_lidar/scan/points");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    raw_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/minco/raw_path", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        jps_max_points_ = std::max(jps_max_points_, message->poses.size());
      });
    preprocessed_guide_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/minco/preprocessed_guide", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        preprocessed_guide_max_points_ = std::max(
          preprocessed_guide_max_points_, message->poses.size());
      });
    esdf_refined_guide_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/minco/esdf_refined_guide", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        esdf_refined_guide_max_points_ = std::max(
          esdf_refined_guide_max_points_, message->poses.size());
      });
    reference_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/minco/reference_path", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        minco_max_points_ = std::max(minco_max_points_, message->poses.size());
      });
    predicted_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/ats_swerve_mpc/predicted_path", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        predicted_max_points_ = std::max(predicted_max_points_, message->poses.size());
      });
    executed_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/ats_swerve_mpc/executed_path", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        executed_max_points_ = std::max(executed_max_points_, message->poses.size());
      });
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel/selected", path_qos,
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        const bool command_nonzero = nonzero(*message);
        selected_cmd_vel_nonzero_ = selected_cmd_vel_nonzero_ || command_nonzero;
        // This recorder is a read-only arming witness for the recovery test.
        // It exits after the first command actually observed by DDS, allowing
        // the shell to cancel the active action without polling ros2cli.
        if (exit_on_nonzero_command_ && command_nonzero && !nonzero_command_triggered_) {
          nonzero_command_triggered_ = true;
          deadline_timer_->cancel();
          rclcpp::shutdown();
        }
      });
    gazebo_lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/" + robot_name_ + "/livox/lidar", sensor_qos,
      [this](
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
        const rclcpp::MessageInfo & message_info) {
        observePointCloud(gazebo_lidar_arrivals_, message->header.stamp);
        observeGazeboLidarPublicationSequence(message_info);
      });
    localization_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/localization", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        observeOdometry(localization_arrivals_, *message);
      });
    lidar_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar_odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        observeOdometry(lidar_odometry_arrivals_, *message);
      });
    cloud_registered_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", sensor_qos,
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        observePointCloud(cloud_registered_arrivals_, message->header.stamp);
      });
    livox_input_sub_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar", sensor_qos,
      [this](const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message) {
        observePointCloud(livox_input_arrivals_, message->header.stamp);
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        observeOdometry(odometry_arrivals_, *message);
      });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(10).best_effort(),
      [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
        const auto callback_started = SteadyClock::now();
        const auto receipt = SteadyClock::now();
        clock_arrivals_.observeReceipt(receipt);
        const auto stamp_ns = toNanoseconds(message->clock);
        clock_arrivals_.observeStamp(stamp_ns);
        if (previous_clock_stamp_ns_ && previous_clock_receipt_) {
          const auto wall_delta = std::chrono::duration<double>(
            receipt - *previous_clock_receipt_).count();
          const auto sim_delta = static_cast<double>(stamp_ns - *previous_clock_stamp_ns_) * 1e-9;
          if (wall_delta > 0.0 && sim_delta >= 0.0) {
            rtf_samples_.push_back(sim_delta / wall_delta);
          }
        }
        previous_clock_stamp_ns_ = stamp_ns;
        previous_clock_receipt_ = receipt;
        latest_clock_stamp_ns_ = stamp_ns;
        // The dynamic-TF freshness witness needs the consumer's reference
        // clock to measure age and staleness, so it must see every /clock
        // sample, not only the ones the arrival statistics keep.
        tf_freshness_.observeClock(stamp_ns);
        clock_arrivals_.observeCallbackDuration(callback_started);
      });
    localization_status_sub_ = create_subscription<
      ats_navigation_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(10).reliable(),
      [this](const ats_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr message) {
        const auto callback_started = SteadyClock::now();
        const auto receipt = SteadyClock::now();
        localization_status_arrivals_.observeReceipt(receipt);
        const auto stamp_ns = toNanoseconds(message->header.stamp);
        localization_status_arrivals_.observeStamp(stamp_ns);
        localization_status_arrivals_.observeAge(latest_clock_stamp_ns_, stamp_ns);
        if (std::isfinite(message->observation_age_sec)) {
          localization_status_observation_age_sec_.push_back(message->observation_age_sec);
        }
        localization_status_last_state_ = message->state;
        ++localization_status_samples_;
        if (message->state ==
          ats_navigation_interfaces::msg::LocalizationStatus::STATE_TRACKING)
        {
          ++localization_status_tracking_samples_;
        } else {
          ++localization_status_non_tracking_samples_;
        }
        localization_status_arrivals_.observeCallbackDuration(callback_started);
      });
    map_status_sub_ = create_subscription<ats_navigation_interfaces::msg::PlanningMapStatus>(
      "/rog_map_adapter/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const ats_navigation_interfaces::msg::PlanningMapStatus::ConstSharedPtr message) {
        const auto callback_started = SteadyClock::now();
        observeInterval(last_map_status_received_, map_status_max_interval_sec_);
        map_status_ready_seen_ = map_status_ready_seen_ || message->ready;
        if (message->ready) {
          adapter_source_generation_begin_ = adapter_source_generation_begin_.value_or(
            message->rog_generation);
          adapter_source_generation_end_ = message->rog_generation;
          adapter_publication_sequence_begin_ = adapter_publication_sequence_begin_.value_or(
            message->publication_sequence);
          adapter_publication_sequence_end_ = message->publication_sequence;
        }
        adapter_status_callback_durations_sec_.push_back(
          std::chrono::duration<double>(SteadyClock::now() - callback_started).count());
      });

    if (observe_gazebo_transport_lidar_) {
      const bool subscribed = gazebo_transport_node_.Subscribe(
        gazebo_lidar_transport_topic_,
        &NavigationEvidenceRecorder::onGazeboTransportLidar, this);
      {
        std::lock_guard<std::mutex> lock(gazebo_transport_lidar_mutex_);
        gazebo_transport_lidar_subscription_established_ = subscribed;
      }
      if (subscribed) {
        RCLCPP_INFO(
          get_logger(), "Observing Gazebo Transport LiDAR '%s' for diagnostic evidence.",
          gazebo_lidar_transport_topic_.c_str());
      } else {
        RCLCPP_ERROR(
          get_logger(), "Unable to subscribe to Gazebo Transport LiDAR '%s'.",
          gazebo_lidar_transport_topic_.c_str());
      }
    }

    started_ = SteadyClock::now();
    // A one-shot timer may be dispatched a few milliseconds before its
    // requested steady-clock deadline. Polling the same steady elapsed time
    // prevents a nominally completed observer from being rejected as a
    // 59.98 s sample for a required 60 s window.
    deadline_timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        const auto elapsed = std::chrono::duration<double>(SteadyClock::now() - started_);
        if (elapsed.count() < duration_sec_) {
          return;
        }
        completed_normally_ = true;
        deadline_timer_->cancel();
        rclcpp::shutdown();
      });
    // Capture graph ownership while the action is still live. A shell ros2cli
    // query after result handling races process teardown and can see zero
    // writers even after this recorder received non-zero commands.
    graph_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NavigationEvidenceRecorder::observeGraph, this));
    observeGraph();
  }

  void printResult() const
  {
    const auto ended = SteadyClock::now();
    const double duration = std::chrono::duration<double>(ended - started_).count();
    std::cout << std::fixed << std::setprecision(6)
              << "ATS_NAVIGATION_EVIDENCE_RESULT"
              << " completed=" << (completed_normally_ ? "yes" : "interrupted")
              << " duration_s=" << duration
              << " jps_max_points=" << jps_max_points_
              << " preprocessed_guide_max_points=" << preprocessed_guide_max_points_
              << " esdf_refined_guide_max_points=" << esdf_refined_guide_max_points_
              << " minco_max_points=" << minco_max_points_
              << " mpc_predicted_max_points=" << predicted_max_points_
              << " mpc_executed_max_points=" << executed_max_points_
              << " exit_on_nonzero_command=" << yesNo(exit_on_nonzero_command_)
              << " nonzero_command_triggered=" << yesNo(nonzero_command_triggered_)
              << " selected_cmd_vel_nonzero=" << yesNo(selected_cmd_vel_nonzero_)
              << " selected_cmd_vel_publisher_max=" << selected_cmd_vel_publisher_max_
              << " selected_cmd_vel_subscriber_max=" << selected_cmd_vel_subscriber_max_
              << " planning_grid_publisher_max=" << planning_grid_publisher_max_
              << " planning_grid_subscriber_max=" << planning_grid_subscriber_max_
              << " planning_grid_publisher_names=" << planningGridPublishers()
              << " planning_grid_adapter_seen=" << yesNo(planning_grid_adapter_seen_)
              << " planning_grid_named_non_adapter_seen=" << yesNo(
                   planning_grid_named_non_adapter_seen_)
              << " planning_grid_anonymous_endpoint_seen=" << yesNo(
                   planning_grid_anonymous_endpoint_seen_)
              << gazeboTransportLidarStatistics()
              << arrivalStatistics("gazebo_lidar", gazebo_lidar_arrivals_)
              << gazeboLidarDdsSequenceStatistics()
              << arrivalStatistics("lidar_odometry", lidar_odometry_arrivals_)
              << arrivalStatistics("livox_input", livox_input_arrivals_)
              << arrivalStatistics("cloud_registered", cloud_registered_arrivals_)
              << arrivalStatistics("odometry", odometry_arrivals_)
              << arrivalStatistics("localization", localization_arrivals_)
              << arrivalStatistics("clock", clock_arrivals_)
              << " clock_rtf_p50=" << percentile(rtf_samples_, 0.50)
              << " clock_rtf_p95=" << percentile(rtf_samples_, 0.95)
              << " clock_rtf_p99=" << percentile(rtf_samples_, 0.99)
              << arrivalStatistics("localization_status", localization_status_arrivals_)
              << " localization_status_samples=" << localization_status_samples_
              << " localization_status_tracking_samples=" << localization_status_tracking_samples_
              << " localization_status_non_tracking_samples=" << localization_status_non_tracking_samples_
              << " localization_status_last_state=" << static_cast<int>(localization_status_last_state_)
              << " localization_status_observation_age_p50_s=" << percentile(
                   localization_status_observation_age_sec_, 0.50)
              << " localization_status_observation_age_p95_s=" << percentile(
                   localization_status_observation_age_sec_, 0.95)
              << " localization_status_observation_age_p99_s=" << percentile(
                   localization_status_observation_age_sec_, 0.99)
              << " tf_lookup_attempts=" << tf_tracker_.attempts()
              << " tf_lookup_successes=" << tf_tracker_.successes()
              << " tf_lookup_failures=" << tf_tracker_.failures()
              // The recorder polls map -> gimbal_yaw_odom from the moment it
              // spins, which precedes the first localization_fusion
              // map -> odom publication, so leading failures are an absent
              // transform rather than a broken one. Admission judges the
              // post-establishment count, and tf_chain_established
              // distinguishes a healthy run from one where the chain never
              // came up (both report zero post-establishment failures).
              << " tf_chain_established=" << (tf_tracker_.established() ? "yes" : "no")
              << " tf_lookup_failures_before_establishment="
              << tf_tracker_.failuresBeforeEstablishment()
              << " tf_lookup_failures_after_establishment="
              << tf_tracker_.failuresAfterEstablishment()
              << " tf_lookup_max_ms=" << tf_lookup_max_ms_
              // Dynamic-edge freshness for map -> gimbal_yaw_odom, from the
              // source stamp carried by the transform the lookup returned.
              // Every field below is read by a P1 admission gate or by its
              // fail-closed sample-count check; the runner fails closed when
              // an older recorder binary omits any of them.
              << " tf_dynamic_samples=" << tf_freshness_.samples()
              << " tf_dynamic_distinct_stamp_updates="
              << tf_freshness_.distinctStampUpdates()
              << " tf_dynamic_duplicate_stamps=" << tf_freshness_.duplicateStamps()
              << " tf_dynamic_backward_stamps=" << tf_freshness_.backwardStamps()
              << " tf_dynamic_invalid_stamps=" << tf_freshness_.invalidStamps()
              << " tf_dynamic_future_stamps=" << tf_freshness_.futureStamps()
              << " tf_dynamic_age_p50_s=" << tfDynamicAge(0.50)
              << " tf_dynamic_age_p99_s=" << tfDynamicAge(0.99)
              << " tf_dynamic_age_max_s=" << tfDynamicAge(1.00)
              << " tf_dynamic_age_floor_s=" << optionalDouble(
                   tf_freshness_.ageFloorSec())
              << " tf_dynamic_age_samples=" << tf_freshness_.ageSamples()
              << " tf_dynamic_staleness_samples=" << tf_freshness_.stalenessSamples()
              << " tf_dynamic_stamp_staleness_p50_s=" << tfDynamicStaleness(0.50)
              << " tf_dynamic_stamp_staleness_p99_s=" << tfDynamicStaleness(0.99)
              << " tf_dynamic_stamp_staleness_max_s=" << tfDynamicStaleness(1.00)
              << " tf_dynamic_backward_clock_samples="
              << tf_freshness_.backwardClockSamples()
              << " tf_dynamic_update_gap_samples=" << tf_freshness_.updateGapSamples()
              << " tf_dynamic_update_gap_p50_s=" << tfDynamicUpdateGap(0.50)
              << " tf_dynamic_update_gap_p99_s=" << tfDynamicUpdateGap(0.99)
              << " tf_dynamic_update_gap_max_s=" << tfDynamicUpdateGap(1.00)
              << " dds_queue_drop_counter=unverified_no_portable_rmw_counter"
              << " adapter_status_callback_count=" << adapter_status_callback_durations_sec_.size()
              << " adapter_status_callback_p50_s=" << percentile(
                   adapter_status_callback_durations_sec_, 0.50)
              << " adapter_status_callback_p95_s=" << percentile(
                   adapter_status_callback_durations_sec_, 0.95)
              << " adapter_status_callback_p99_s=" << percentile(
                   adapter_status_callback_durations_sec_, 0.99)
              << " adapter_status_callback_max_s=" << percentile(
                   adapter_status_callback_durations_sec_, 1.00)
              << " adapter_ready_seen=" << yesNo(map_status_ready_seen_)
              << " adapter_max_wall_interval_s=" << optionalDouble(
                   map_status_max_interval_sec_)
              << " adapter_source_generation_begin=" << optionalUint(
                   adapter_source_generation_begin_)
              << " adapter_source_generation_end=" << optionalUint(
                   adapter_source_generation_end_)
              << " adapter_publication_sequence_begin=" << optionalUint(
                   adapter_publication_sequence_begin_)
              << " adapter_publication_sequence_end=" << optionalUint(
                   adapter_publication_sequence_end_)
              << std::endl;
  }

private:
  static constexpr double kNonzeroEpsilon = 1e-6;

  static bool nonzero(const geometry_msgs::msg::Twist & message)
  {
    return std::abs(message.linear.x) > kNonzeroEpsilon ||
           std::abs(message.linear.y) > kNonzeroEpsilon ||
           std::abs(message.angular.z) > kNonzeroEpsilon;
  }

  static const char * yesNo(bool value) { return value ? "yes" : "no"; }

  static std::string optionalDouble(const std::optional<double> & value)
  {
    if (!value) {
      return "unverified";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(6) << *value;
    return output.str();
  }

  static std::string optionalUint(const std::optional<std::uint64_t> & value)
  {
    return value ? std::to_string(*value) : "unverified";
  }

  static std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  static std::int64_t toNanoseconds(const ignition::msgs::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec()) * 1000000000LL + stamp.nsec();
  }

  void observeOdometry(EvidenceStatistics & statistics, const nav_msgs::msg::Odometry & message)
  {
    const auto callback_started = SteadyClock::now();
    statistics.observeReceipt();
    const auto stamp_ns = toNanoseconds(message.header.stamp);
    statistics.observeStamp(stamp_ns);
    statistics.observeAge(latest_clock_stamp_ns_, stamp_ns);
    statistics.observeCallbackDuration(callback_started);
  }

  void observePointCloud(
    EvidenceStatistics & statistics, const builtin_interfaces::msg::Time & stamp)
  {
    const auto callback_started = SteadyClock::now();
    statistics.observeReceipt();
    const auto stamp_ns = toNanoseconds(stamp);
    statistics.observeStamp(stamp_ns);
    statistics.observeAge(latest_clock_stamp_ns_, stamp_ns);
    statistics.observeCallbackDuration(callback_started);
  }

  void observeGazeboLidarPublicationSequence(const rclcpp::MessageInfo & message_info)
  {
    if (!gazebo_lidar_publication_sequence_supported_) {
      return;
    }
    gazebo_lidar_arrivals_.observePublicationSequence(
      message_info.get_rmw_message_info().publication_sequence_number);
  }

  void onGazeboTransportLidar(const ignition::msgs::PointCloudPacked & message)
  {
    const auto callback_started = SteadyClock::now();
    std::lock_guard<std::mutex> lock(gazebo_transport_lidar_mutex_);
    gazebo_transport_lidar_arrivals_.observeReceipt();
    gazebo_transport_lidar_arrivals_.observeStamp(toNanoseconds(message.header().stamp()));
    gazebo_transport_lidar_arrivals_.observeCallbackDuration(callback_started);
  }

  std::string gazeboTransportLidarStatistics() const
  {
    std::lock_guard<std::mutex> lock(gazebo_transport_lidar_mutex_);
    std::ostringstream output;
    output << " gazebo_transport_lidar_observation_enabled=" << yesNo(
      observe_gazebo_transport_lidar_)
           << " gazebo_transport_lidar_subscription_established=" << yesNo(
      gazebo_transport_lidar_subscription_established_)
           << " gazebo_transport_lidar_topic=" << gazebo_lidar_transport_topic_
           << arrivalStatistics("gazebo_transport_lidar", gazebo_transport_lidar_arrivals_);
    return output.str();
  }

  std::string gazeboLidarDdsSequenceStatistics() const
  {
    std::ostringstream output;
    output << " gazebo_lidar_dds_publication_sequence_supported=" << yesNo(
      gazebo_lidar_publication_sequence_supported_)
           << " gazebo_lidar_dds_publication_sequence_samples="
           << gazebo_lidar_arrivals_.publication_sequence_samples
           << " gazebo_lidar_dds_publication_sequence_gap_count="
           << gazebo_lidar_arrivals_.publication_sequence_gap_count
           << " gazebo_lidar_dds_publication_sequence_missing_count="
           << gazebo_lidar_arrivals_.publication_sequence_missing_count
           << " gazebo_lidar_dds_publication_sequence_nonmonotonic_count="
           << gazebo_lidar_arrivals_.publication_sequence_nonmonotonic_count;
    return output.str();
  }

  static std::string arrivalStatistics(const std::string & name, const EvidenceStatistics & statistics)
  {
    std::ostringstream output;
    output << ' ' << name << "_samples=" << statistics.samples
           << ' ' << name << "_p50_wall_interval_s=" << percentile(statistics.wall_intervals_sec, 0.50)
           << ' ' << name << "_p95_wall_interval_s=" << percentile(statistics.wall_intervals_sec, 0.95)
           << ' ' << name << "_p99_wall_interval_s=" << percentile(statistics.wall_intervals_sec, 0.99)
           << ' ' << name << "_max_wall_interval_s=" << percentile(statistics.wall_intervals_sec, 1.00)
           << ' ' << name << "_p50_stamp_interval_s=" << percentile(statistics.stamp_intervals_sec, 0.50)
           << ' ' << name << "_p95_stamp_interval_s=" << percentile(statistics.stamp_intervals_sec, 0.95)
           << ' ' << name << "_p99_stamp_interval_s=" << percentile(statistics.stamp_intervals_sec, 0.99)
           << ' ' << name << "_max_stamp_interval_s=" << percentile(statistics.stamp_intervals_sec, 1.00)
           << ' ' << name << "_p50_stamp_age_s=" << percentile(statistics.stamp_ages_sec, 0.50)
           << ' ' << name << "_p95_stamp_age_s=" << percentile(statistics.stamp_ages_sec, 0.95)
           << ' ' << name << "_p99_stamp_age_s=" << percentile(statistics.stamp_ages_sec, 0.99)
           << ' ' << name << "_max_stamp_age_s=" << percentile(statistics.stamp_ages_sec, 1.00)
           << ' ' << name << "_duplicate_stamp_count=" << statistics.duplicate_stamp_count
           << ' ' << name << "_backward_stamp_count=" << statistics.backward_stamp_count
           << ' ' << name << "_invalid_stamp_count=" << statistics.invalid_stamp_count
           << ' ' << name << "_future_stamp_count=" << statistics.future_stamp_count
           << ' ' << name << "_callback_count=" << statistics.callback_durations_sec.size()
           << ' ' << name << "_p50_callback_duration_s=" << percentile(
             statistics.callback_durations_sec, 0.50)
           << ' ' << name << "_p95_callback_duration_s=" << percentile(
             statistics.callback_durations_sec, 0.95)
           << ' ' << name << "_p99_callback_duration_s=" << percentile(
             statistics.callback_durations_sec, 0.99)
           << ' ' << name << "_max_callback_duration_s=" << percentile(
             statistics.callback_durations_sec, 1.00);
    return output.str();
  }

  static std::string percentile(const std::vector<double> & values, const double probability)
  {
    if (values.empty()) {
      return "unverified";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << rmu_gazebo_simulator::percentile(values, probability);
    return output.str();
  }

  // Dynamic-TF percentiles print 0.000000 on an empty sample set, never
  // "unverified": the runner's fail-closed sample-count gates run before any
  // threshold is consulted, and the behavioral suite pins the numeric shape
  // of a no-sample line (an "unverified" token would defeat those gates by
  // parsing as a missing field instead of an empty distribution).
  static std::string tfDynamicPercentile(
    const std::vector<double> & values, const double probability)
  {
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << rmu_gazebo_simulator::percentile(values, probability);
    return output.str();
  }

  std::string tfDynamicAge(const double probability) const
  {
    return tfDynamicPercentile(tf_freshness_.agesSec(), probability);
  }

  std::string tfDynamicStaleness(const double probability) const
  {
    return tfDynamicPercentile(tf_freshness_.stalenessSec(), probability);
  }

  std::string tfDynamicUpdateGap(const double probability) const
  {
    return tfDynamicPercentile(tf_freshness_.updateGapsSec(), probability);
  }

  static void observeInterval(
    std::optional<SteadyTime> & previous, std::optional<double> & maximum)
  {
    const auto receipt = SteadyClock::now();
    if (previous) {
      const double interval = std::chrono::duration<double>(receipt - *previous).count();
      maximum = std::max(maximum.value_or(0.0), interval);
    }
    previous = receipt;
  }

  void observeGraph()
  {
    observeTf();
    try {
      const auto planning_grid_publishers = get_publishers_info_by_topic(
        "/rc_esdf/planning_grid");
      planning_grid_publisher_max_ = std::max(
        planning_grid_publisher_max_, planning_grid_publishers.size());
      for (const auto & publisher : planning_grid_publishers) {
        const std::string publisher_name = endpointName(publisher);
        planning_grid_publishers_.insert(publisher_name);
        if (publisher_name == "/ats_rog_map_adapter") {
          planning_grid_adapter_seen_ = true;
        } else if (isAnonymousEndpoint(publisher_name)) {
          planning_grid_anonymous_endpoint_seen_ = true;
        } else {
          planning_grid_named_non_adapter_seen_ = true;
        }
      }
      planning_grid_subscriber_max_ = std::max(
        planning_grid_subscriber_max_, count_subscribers("/rc_esdf/planning_grid"));
      selected_cmd_vel_publisher_max_ = std::max(
        selected_cmd_vel_publisher_max_, count_publishers("/cmd_vel/selected"));
      selected_cmd_vel_subscriber_max_ = std::max(
        selected_cmd_vel_subscriber_max_, count_subscribers("/cmd_vel/selected"));
    } catch (const std::exception &) {
      // Shutdown can cancel the timer concurrently. Evidence collected before
      // that point remains valid and is printed after spin returns.
    }
  }

  void observeTf()
  {
    if (!tf_buffer_) {
      return;
    }
    const auto started = SteadyClock::now();
    try {
      // The dynamic freshness witness must judge the source stamp of the
      // transform this lookup returns (transform.header.stamp). A
      // TimePointZero query whose result is discarded proves only that the
      // chain resolves, which is exactly the gap the freshness evidence
      // exists to close: a broadcaster that died leaves the buffer replaying
      // its last transform, and only the carried stamp keeps advancing with
      // the reference clock in a live chain.
      const geometry_msgs::msg::TransformStamped transform =
        tf_buffer_->lookupTransform("map", "gimbal_yaw_odom", tf2::TimePointZero);
      tf_tracker_.observeSuccess();
      tf_freshness_.observeLookup(toNanoseconds(transform.header.stamp));
    } catch (const tf2::TransformException &) {
      tf_tracker_.observeFailure();
    }
    tf_lookup_max_ms_ = std::max(
      tf_lookup_max_ms_,
      std::chrono::duration<double, std::milli>(SteadyClock::now() - started).count());
  }

  static std::string endpointName(const rclcpp::TopicEndpointInfo & endpoint)
  {
    const std::string node_namespace = endpoint.node_namespace();
    if (node_namespace.empty() || node_namespace == "/") {
      return "/" + endpoint.node_name();
    }
    return node_namespace + "/" + endpoint.node_name();
  }

  static bool isAnonymousEndpoint(const std::string & endpoint_name)
  {
    return endpoint_name.find("_NODE_NAMESPACE_UNKNOWN_") != std::string::npos ||
           endpoint_name.find("_NODE_NAME_UNKNOWN_") != std::string::npos;
  }

  std::string planningGridPublishers() const
  {
    if (planning_grid_publishers_.empty()) {
      return "unverified";
    }
    std::ostringstream output;
    for (auto iter = planning_grid_publishers_.begin();
      iter != planning_grid_publishers_.end(); ++iter)
    {
      if (iter != planning_grid_publishers_.begin()) {
        output << ',';
      }
      output << *iter;
    }
    return output.str();
  }

  double duration_sec_{30.0};
  std::string robot_name_;
  SteadyTime started_;
  bool completed_normally_{false};
  std::size_t jps_max_points_{0};
  std::size_t preprocessed_guide_max_points_{0};
  std::size_t esdf_refined_guide_max_points_{0};
  std::size_t minco_max_points_{0};
  std::size_t predicted_max_points_{0};
  std::size_t executed_max_points_{0};
  bool exit_on_nonzero_command_{false};
  bool nonzero_command_triggered_{false};
  bool selected_cmd_vel_nonzero_{false};
  std::size_t selected_cmd_vel_publisher_max_{0};
  std::size_t selected_cmd_vel_subscriber_max_{0};
  std::size_t planning_grid_publisher_max_{0};
  std::size_t planning_grid_subscriber_max_{0};
  std::set<std::string> planning_grid_publishers_;
  bool planning_grid_adapter_seen_{false};
  bool planning_grid_named_non_adapter_seen_{false};
  bool planning_grid_anonymous_endpoint_seen_{false};
  bool observe_gazebo_transport_lidar_{false};
  const bool gazebo_lidar_publication_sequence_supported_{
    rmw_feature_supported(RMW_FEATURE_MESSAGE_INFO_PUBLICATION_SEQUENCE_NUMBER)};
  std::string gazebo_lidar_transport_topic_;
  ignition::transport::Node gazebo_transport_node_;
  mutable std::mutex gazebo_transport_lidar_mutex_;
  bool gazebo_transport_lidar_subscription_established_{false};
  EvidenceStatistics gazebo_transport_lidar_arrivals_;
  EvidenceStatistics gazebo_lidar_arrivals_;
  EvidenceStatistics lidar_odometry_arrivals_;
  EvidenceStatistics livox_input_arrivals_;
  EvidenceStatistics cloud_registered_arrivals_;
  EvidenceStatistics odometry_arrivals_;
  EvidenceStatistics localization_arrivals_;
  EvidenceStatistics clock_arrivals_;
  EvidenceStatistics localization_status_arrivals_;
  std::vector<double> rtf_samples_;
  std::optional<std::int64_t> latest_clock_stamp_ns_;
  std::optional<std::int64_t> previous_clock_stamp_ns_;
  std::optional<SteadyTime> previous_clock_receipt_;
  std::size_t localization_status_samples_{0};
  std::size_t localization_status_tracking_samples_{0};
  std::size_t localization_status_non_tracking_samples_{0};
  std::vector<double> localization_status_observation_age_sec_;
  std::uint8_t localization_status_last_state_{
    ats_navigation_interfaces::msg::LocalizationStatus::STATE_UNINITIALIZED};
  TfEstablishmentTracker tf_tracker_;
  DynamicTransformFreshness tf_freshness_;
  double tf_lookup_max_ms_{0.0};
  bool map_status_ready_seen_{false};
  std::optional<SteadyTime> last_map_status_received_;
  std::optional<double> map_status_max_interval_sec_;
  std::optional<std::uint64_t> adapter_source_generation_begin_;
  std::optional<std::uint64_t> adapter_source_generation_end_;
  std::optional<std::uint64_t> adapter_publication_sequence_begin_;
  std::optional<std::uint64_t> adapter_publication_sequence_end_;
  std::vector<double> adapter_status_callback_durations_sec_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr preprocessed_guide_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr esdf_refined_guide_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr predicted_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr executed_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr gazebo_lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_sub_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_input_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_registered_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_status_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlanningMapStatus>::SharedPtr
    map_status_sub_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<NavigationEvidenceRecorder>();
  rclcpp::spin(node);
  node->printResult();
  return 0;
}
