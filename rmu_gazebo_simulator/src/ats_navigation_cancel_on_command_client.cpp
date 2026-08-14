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

// A purpose-built recovery-test client. It owns the action goal it sends and
// cancels that exact goal from the first observed non-zero MPC command. Keeping
// the command subscription and action GoalHandle in one executor removes the
// process/DDS race of a shell `ros2 topic echo` followed by SIGINT.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <ats_navigation_interfaces/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace
{

using NavigateToPose = ats_navigation_interfaces::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class CancelOnCommandClient final : public rclcpp::Node
{
public:
  CancelOnCommandClient()
  : Node(
      "ats_navigation_cancel_on_command_client",
      rclcpp::NodeOptions().append_parameter_override("use_sim_time", false))
  {
    action_name_ = declare_parameter<std::string>("action_name", "/ats_navigate_to_pose");
    goal_frame_ = declare_parameter<std::string>("goal_frame", "map");
    goal_x_ = declare_parameter<double>("goal_x", 2.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_yaw_ = declare_parameter<double>("goal_yaw", 0.0);
    goal_timeout_sec_ = std::max(0.1, declare_parameter<double>("goal_timeout_sec", 120.0));
    wall_timeout_sec_ = std::max(0.1, declare_parameter<double>("wall_timeout_sec", 20.0));

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_mpc", command_qos,
      std::bind(&CancelOnCommandClient::onCommand, this, std::placeholders::_1));
    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    server_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CancelOnCommandClient::trySendGoal, this));
    deadline_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(wall_timeout_sec_)),
      [this]() {
        timed_out_ = true;
        finish();
      });
  }

  bool passed() const
  {
    return goal_accepted_ && command_nonzero_ && cancel_request_sent_ &&
           cancel_response_received_ && cancel_accepted_ && result_received_ &&
           result_code_ == rclcpp_action::ResultCode::CANCELED &&
           action_result_code_ == NavigateToPose::Result::RESULT_CANCELED && !timed_out_;
  }

  void printResult() const
  {
    std::cout << "ATS_CANCEL_ON_COMMAND_RESULT"
              << " goal_sent=" << yesNo(goal_sent_)
              << " goal_accepted=" << yesNo(goal_accepted_)
              << " cmd_vel_nonzero=" << yesNo(command_nonzero_)
              << " cancel_request_sent=" << yesNo(cancel_request_sent_)
              << " cancel_response_received=" << yesNo(cancel_response_received_)
              << " cancel_accepted=" << yesNo(cancel_accepted_)
              << " action_result=" << resultName(result_code_)
              << " action_result_code=" << static_cast<unsigned int>(action_result_code_)
              << " timed_out=" << yesNo(timed_out_)
              << " passed=" << yesNo(passed())
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

  static const char * resultName(rclcpp_action::ResultCode result)
  {
    switch (result) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        return "SUCCEEDED";
      case rclcpp_action::ResultCode::ABORTED:
        return "ABORTED";
      case rclcpp_action::ResultCode::CANCELED:
        return "CANCELED";
      default:
        return "UNKNOWN";
    }
  }

  void trySendGoal()
  {
    if (goal_sent_ || !action_client_->wait_for_action_server(std::chrono::seconds(0))) {
      return;
    }
    goal_sent_ = true;
    server_timer_->cancel();

    NavigateToPose::Goal goal;
    goal.goal_pose.header.frame_id = goal_frame_;
    goal.goal_pose.pose.position.x = goal_x_;
    goal.goal_pose.pose.position.y = goal_y_;
    goal.goal_pose.pose.orientation.z = std::sin(goal_yaw_ * 0.5);
    goal.goal_pose.pose.orientation.w = std::cos(goal_yaw_ * 0.5);
    const auto timeout_ns = static_cast<std::int64_t>(goal_timeout_sec_ * 1e9);
    goal.timeout.sec = static_cast<std::int32_t>(timeout_ns / 1000000000LL);
    goal.timeout.nanosec = static_cast<std::uint32_t>(timeout_ns % 1000000000LL);

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr handle) {
      if (!handle) {
        finish();
        return;
      }
      goal_accepted_ = true;
      goal_handle_ = std::move(handle);
    };
    options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      result_received_ = true;
      result_code_ = result.code;
      if (result.result) {
        action_result_code_ = result.result->result_code;
      }
      maybeFinishAfterActionEvent();
    };
    action_client_->async_send_goal(goal, options);
  }

  void onCommand(const geometry_msgs::msg::Twist::ConstSharedPtr message)
  {
    if (!nonzero(*message)) {
      return;
    }
    command_nonzero_ = true;
    if (!goal_handle_ || cancel_request_sent_) {
      return;
    }
    cancel_request_sent_ = true;
    try {
      action_client_->async_cancel_goal(
        goal_handle_, [this](rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr response) {
          cancel_response_received_ = true;
          cancel_accepted_ = response && !response->goals_canceling.empty();
          maybeFinishAfterActionEvent();
        });
    } catch (const std::exception &) {
      cancel_response_received_ = true;
      cancel_accepted_ = false;
      finish();
    }
  }

  void maybeFinishAfterActionEvent()
  {
    if (result_received_ && (!cancel_request_sent_ || cancel_response_received_)) {
      finish();
    }
  }

  void finish()
  {
    if (completed_) {
      return;
    }
    completed_ = true;
    server_timer_->cancel();
    deadline_timer_->cancel();
    rclcpp::shutdown();
  }

  std::string action_name_;
  std::string goal_frame_;
  double goal_x_{2.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double goal_timeout_sec_{120.0};
  double wall_timeout_sec_{20.0};
  bool goal_sent_{false};
  bool goal_accepted_{false};
  bool command_nonzero_{false};
  bool cancel_request_sent_{false};
  bool cancel_response_received_{false};
  bool cancel_accepted_{false};
  bool result_received_{false};
  bool timed_out_{false};
  bool completed_{false};
  rclcpp_action::ResultCode result_code_{rclcpp_action::ResultCode::UNKNOWN};
  std::uint8_t action_result_code_{255U};
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr server_timer_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<CancelOnCommandClient>();
  rclcpp::spin(node);
  node->printResult();
  return node->passed() ? 0 : 1;
}
