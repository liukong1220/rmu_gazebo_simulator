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
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <ats_navigation_interfaces/msg/planning_map_status.hpp>
#include <ats_navigation_interfaces/msg/localization_status.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <manda_can_control/msg/motion_ctrl.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rmu_gazebo_simulator/evidence_statistics.hpp"

namespace
{

using rmu_gazebo_simulator::EvidenceStatistics;
using rmu_gazebo_simulator::SteadyClock;
using rmu_gazebo_simulator::SteadyTime;

struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

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
      "/cmd_vel_mpc", path_qos,
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        const bool command_nonzero = nonzero(*message);
        cmd_vel_nonzero_ = cmd_vel_nonzero_ || command_nonzero;
        // This recorder is a read-only arming witness for the recovery test.
        // It exits after the first command actually observed by DDS, allowing
        // the shell to cancel the active action without polling ros2cli.
        if (exit_on_nonzero_command_ && command_nonzero && !nonzero_command_triggered_) {
          nonzero_command_triggered_ = true;
          deadline_timer_->cancel();
          rclcpp::shutdown();
        }
      });
    motion_sub_ = create_subscription<manda_can_control::msg::MotionCtrl>(
      "/motion_control", path_qos,
      [this](const manda_can_control::msg::MotionCtrl::ConstSharedPtr message) {
        motion_control_nonzero_ = motion_control_nonzero_ ||
          std::abs(message->linear_x) > kNonzeroEpsilon ||
          std::abs(message->linear_y) > kNonzeroEpsilon ||
          std::abs(message->angular_z) > kNonzeroEpsilon;
      });
    chassis_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/" + robot_name_ + "/cmd_vel", path_qos,
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr message) {
        chassis_command_nonzero_ = chassis_command_nonzero_ || nonzero(*message);
      });
    ground_truth_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_name_ + "/chassis_odometry_gt", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        PoseSample sample;
        sample.x = message->pose.pose.position.x;
        sample.y = message->pose.pose.position.y;
        sample.z = message->pose.pose.position.z;
        if (!ground_truth_begin_) {
          ground_truth_begin_ = sample;
        }
        ground_truth_end_ = sample;
      });
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/" + robot_name_ + "/joint_states", sensor_qos,
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        recordJointState(*message);
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
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr message) {
        observeOdometry(odometry_arrivals_, *message);
      });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(10).best_effort(),
      [this](const rosgraph_msgs::msg::Clock::ConstSharedPtr message) {
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
      });
    localization_status_sub_ = create_subscription<
      ats_navigation_interfaces::msg::LocalizationStatus>(
      "/localization/status", rclcpp::QoS(10).reliable(),
      [this](const ats_navigation_interfaces::msg::LocalizationStatus::ConstSharedPtr message) {
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
      });
    map_status_sub_ = create_subscription<ats_navigation_interfaces::msg::PlanningMapStatus>(
      "/rog_map_adapter/status", rclcpp::QoS(1).reliable().transient_local(),
      [this](const ats_navigation_interfaces::msg::PlanningMapStatus::ConstSharedPtr message) {
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
      });

    started_ = SteadyClock::now();
    deadline_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(duration_sec_)),
      [this]() {
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
              << " cmd_vel_nonzero=" << yesNo(cmd_vel_nonzero_)
              << " motion_control_nonzero=" << yesNo(motion_control_nonzero_)
              << " chassis_cmd_nonzero=" << yesNo(chassis_command_nonzero_)
              << " cmd_vel_mpc_publisher_max=" << cmd_vel_mpc_publisher_max_
              << " cmd_vel_mpc_subscriber_max=" << cmd_vel_mpc_subscriber_max_
              << " motion_control_publisher_max=" << motion_control_publisher_max_
              << " motion_control_subscriber_max=" << motion_control_subscriber_max_
              << " chassis_cmd_publisher_max=" << chassis_cmd_publisher_max_
              << " chassis_cmd_subscriber_max=" << chassis_cmd_subscriber_max_
              << " planning_grid_publisher_max=" << planning_grid_publisher_max_
              << " planning_grid_subscriber_max=" << planning_grid_subscriber_max_
              << " planning_grid_publisher_names=" << planningGridPublishers()
              << " planning_grid_adapter_seen=" << yesNo(planning_grid_adapter_seen_)
              << " planning_grid_named_non_adapter_seen=" << yesNo(
                   planning_grid_named_non_adapter_seen_)
              << " planning_grid_anonymous_endpoint_seen=" << yesNo(
                   planning_grid_anonymous_endpoint_seen_)
              << " wheel_active=" << yesNo(wheel_active_)
              << " wheel_peak_rad_s=" << wheelPeaks()
              << " gt_begin_xyz=" << poseValue(ground_truth_begin_)
              << " gt_end_xyz=" << poseValue(ground_truth_end_)
              << arrivalStatistics("lidar_odometry", lidar_odometry_arrivals_)
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
              << " tf_lookup_attempts=" << tf_lookup_attempts_
              << " tf_lookup_successes=" << tf_lookup_successes_
              << " tf_lookup_failures=" << tf_lookup_failures_
              << " tf_lookup_max_ms=" << tf_lookup_max_ms_
              << " dds_queue_drop_counter=unverified_no_portable_rmw_counter"
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

  static std::string poseValue(const std::optional<PoseSample> & value)
  {
    if (!value) {
      return "unverified";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(6)
           << value->x << ',' << value->y << ',' << value->z;
    return output.str();
  }

  static std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  void observeOdometry(EvidenceStatistics & statistics, const nav_msgs::msg::Odometry & message)
  {
    statistics.observeReceipt();
    const auto stamp_ns = toNanoseconds(message.header.stamp);
    statistics.observeStamp(stamp_ns);
    statistics.observeAge(latest_clock_stamp_ns_, stamp_ns);
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
           << ' ' << name << "_future_stamp_count=" << statistics.future_stamp_count;
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
      cmd_vel_mpc_publisher_max_ = std::max(
        cmd_vel_mpc_publisher_max_, count_publishers("/cmd_vel_mpc"));
      cmd_vel_mpc_subscriber_max_ = std::max(
        cmd_vel_mpc_subscriber_max_, count_subscribers("/cmd_vel_mpc"));
      motion_control_publisher_max_ = std::max(
        motion_control_publisher_max_, count_publishers("/motion_control"));
      motion_control_subscriber_max_ = std::max(
        motion_control_subscriber_max_, count_subscribers("/motion_control"));
      chassis_cmd_publisher_max_ = std::max(
        chassis_cmd_publisher_max_, count_publishers("/" + robot_name_ + "/cmd_vel"));
      chassis_cmd_subscriber_max_ = std::max(
        chassis_cmd_subscriber_max_, count_subscribers("/" + robot_name_ + "/cmd_vel"));
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
    ++tf_lookup_attempts_;
    const auto started = SteadyClock::now();
    try {
      (void)tf_buffer_->lookupTransform("map", "gimbal_yaw_odom", tf2::TimePointZero);
      ++tf_lookup_successes_;
    } catch (const tf2::TransformException &) {
      ++tf_lookup_failures_;
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

  void recordJointState(const sensor_msgs::msg::JointState & message)
  {
    const std::size_t count = std::min(message.name.size(), message.velocity.size());
    for (std::size_t index = 0; index < count; ++index) {
      const auto & name = message.name[index];
      if (!isSwerveJoint(name)) {
        continue;
      }
      const double speed = std::abs(message.velocity[index]);
      if (!std::isfinite(speed)) {
        continue;
      }
      wheel_peak_rad_s_[name] = std::max(wheel_peak_rad_s_[name], speed);
      if (isWheelJoint(name) && speed > kNonzeroEpsilon) {
        wheel_active_ = true;
      }
    }
  }

  static bool isSwerveJoint(const std::string & name)
  {
    return name == "front_left_steer_joint" || name == "front_right_steer_joint" ||
           name == "rear_left_steer_joint" || name == "rear_right_steer_joint" ||
           isWheelJoint(name);
  }

  static bool isWheelJoint(const std::string & name)
  {
    return name == "front_left_wheel_joint" || name == "front_right_wheel_joint" ||
           name == "rear_left_wheel_joint" || name == "rear_right_wheel_joint";
  }

  std::string wheelPeaks() const
  {
    static const char * const names[] = {
      "front_left_steer_joint", "front_right_steer_joint", "rear_left_steer_joint",
      "rear_right_steer_joint", "front_left_wheel_joint", "front_right_wheel_joint",
      "rear_left_wheel_joint", "rear_right_wheel_joint"};
    std::ostringstream output;
    output << std::fixed << std::setprecision(6);
    for (std::size_t index = 0; index < std::size(names); ++index) {
      if (index > 0) {
        output << ',';
      }
      const auto found = wheel_peak_rad_s_.find(names[index]);
      output << names[index] << ':' << (found == wheel_peak_rad_s_.end() ? 0.0 : found->second);
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
  bool cmd_vel_nonzero_{false};
  bool motion_control_nonzero_{false};
  bool chassis_command_nonzero_{false};
  std::size_t cmd_vel_mpc_publisher_max_{0};
  std::size_t cmd_vel_mpc_subscriber_max_{0};
  std::size_t motion_control_publisher_max_{0};
  std::size_t motion_control_subscriber_max_{0};
  std::size_t chassis_cmd_publisher_max_{0};
  std::size_t chassis_cmd_subscriber_max_{0};
  std::size_t planning_grid_publisher_max_{0};
  std::size_t planning_grid_subscriber_max_{0};
  std::set<std::string> planning_grid_publishers_;
  bool planning_grid_adapter_seen_{false};
  bool planning_grid_named_non_adapter_seen_{false};
  bool planning_grid_anonymous_endpoint_seen_{false};
  bool wheel_active_{false};
  std::optional<PoseSample> ground_truth_begin_;
  std::optional<PoseSample> ground_truth_end_;
  EvidenceStatistics lidar_odometry_arrivals_;
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
  std::size_t tf_lookup_attempts_{0};
  std::size_t tf_lookup_successes_{0};
  std::size_t tf_lookup_failures_{0};
  double tf_lookup_max_ms_{0.0};
  bool map_status_ready_seen_{false};
  std::optional<SteadyTime> last_map_status_received_;
  std::optional<double> map_status_max_interval_sec_;
  std::optional<std::uint64_t> adapter_source_generation_begin_;
  std::optional<std::uint64_t> adapter_source_generation_end_;
  std::optional<std::uint64_t> adapter_publication_sequence_begin_;
  std::optional<std::uint64_t> adapter_publication_sequence_end_;
  std::map<std::string, double> wheel_peak_rad_s_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr preprocessed_guide_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr esdf_refined_guide_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr reference_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr predicted_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr executed_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<manda_can_control::msg::MotionCtrl>::SharedPtr motion_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_sub_;
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
