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
#include <sstream>
#include <string>
#include <vector>

#include <ats_navigation_interfaces/msg/planning_map_status.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <manda_can_control/msg/motion_ctrl.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace
{

using SteadyClock = std::chrono::steady_clock;
using SteadyTime = SteadyClock::time_point;

struct PoseSample
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct ArrivalStatistics
{
  std::size_t samples{0};
  std::optional<SteadyTime> previous_receipt;
  std::vector<double> wall_intervals_sec;

  void observe()
  {
    const auto receipt = SteadyClock::now();
    if (previous_receipt) {
      wall_intervals_sec.push_back(
        std::chrono::duration<double>(receipt - *previous_receipt).count());
    }
    previous_receipt = receipt;
    ++samples;
  }
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

    const auto path_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto sensor_qos = rclcpp::SensorDataQoS();

    raw_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "/minco/raw_path", path_qos,
      [this](const nav_msgs::msg::Path::ConstSharedPtr message) {
        jps_max_points_ = std::max(jps_max_points_, message->poses.size());
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
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr) {
        localization_arrivals_.observe();
      });
    lidar_odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/lidar_odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr) {
        lidar_odometry_arrivals_.observe();
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", sensor_qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr) {
        odometry_arrivals_.observe();
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
              << " wheel_active=" << yesNo(wheel_active_)
              << " wheel_peak_rad_s=" << wheelPeaks()
              << " gt_begin_xyz=" << poseValue(ground_truth_begin_)
              << " gt_end_xyz=" << poseValue(ground_truth_end_)
              << arrivalStatistics("lidar_odometry", lidar_odometry_arrivals_)
              << arrivalStatistics("odometry", odometry_arrivals_)
              << arrivalStatistics("localization", localization_arrivals_)
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

  static std::string arrivalStatistics(const std::string & name, const ArrivalStatistics & statistics)
  {
    std::ostringstream output;
    output << ' ' << name << "_samples=" << statistics.samples
           << ' ' << name << "_p50_wall_interval_s=" << percentile(statistics, 0.50)
           << ' ' << name << "_p95_wall_interval_s=" << percentile(statistics, 0.95)
           << ' ' << name << "_p99_wall_interval_s=" << percentile(statistics, 0.99)
           << ' ' << name << "_max_wall_interval_s=" << percentile(statistics, 1.00);
    return output.str();
  }

  static std::string percentile(const ArrivalStatistics & statistics, double probability)
  {
    if (statistics.wall_intervals_sec.empty()) {
      return "unverified";
    }
    auto sorted = statistics.wall_intervals_sec;
    std::sort(sorted.begin(), sorted.end());
    const auto index = static_cast<std::size_t>(std::ceil(
      std::clamp(probability, 0.0, 1.0) * static_cast<double>(sorted.size()))) - 1U;
    std::ostringstream output;
    output << std::fixed << std::setprecision(6) << sorted[index];
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
    try {
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
  bool wheel_active_{false};
  std::optional<PoseSample> ground_truth_begin_;
  std::optional<PoseSample> ground_truth_end_;
  ArrivalStatistics lidar_odometry_arrivals_;
  ArrivalStatistics odometry_arrivals_;
  ArrivalStatistics localization_arrivals_;
  bool map_status_ready_seen_{false};
  std::optional<SteadyTime> last_map_status_received_;
  std::optional<double> map_status_max_interval_sec_;
  std::optional<std::uint64_t> adapter_source_generation_begin_;
  std::optional<std::uint64_t> adapter_source_generation_end_;
  std::optional<std::uint64_t> adapter_publication_sequence_begin_;
  std::optional<std::uint64_t> adapter_publication_sequence_end_;
  std::map<std::string, double> wheel_peak_rad_s_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
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
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlanningMapStatus>::SharedPtr
    map_status_sub_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
  rclcpp::TimerBase::SharedPtr graph_timer_;
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
