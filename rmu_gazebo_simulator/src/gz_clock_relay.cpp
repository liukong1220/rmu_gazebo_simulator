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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include <ignition/msgs/clock.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace
{

class GazeboClockRelay final : public rclcpp::Node
{
public:
  GazeboClockRelay()
  : Node("gz_clock_relay")
  {
    const auto gazebo_clock_topic = declare_parameter<std::string>(
      "gazebo_clock_topic", "/clock");
    const auto ros_clock_topic = declare_parameter<std::string>("ros_clock_topic", "/clock");
    const auto max_publish_rate_hz = std::max(
      0.0, declare_parameter<double>("max_publish_rate_hz", 100.0));

    if (max_publish_rate_hz > 0.0) {
      min_publish_interval_ns_ = static_cast<std::int64_t>(std::llround(
        1.0e9 / max_publish_rate_hz));
    }

    clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
      ros_clock_topic, rclcpp::ClockQoS());
    if (!transport_node_.Subscribe(gazebo_clock_topic, &GazeboClockRelay::onGazeboClock, this)) {
      throw std::runtime_error("Unable to subscribe to Gazebo clock topic '" + gazebo_clock_topic + "'");
    }

    RCLCPP_INFO(
      get_logger(),
      "Relaying Gazebo clock '%s' to ROS '%s' at most %.1f Hz",
      gazebo_clock_topic.c_str(), ros_clock_topic.c_str(), max_publish_rate_hz);
  }

private:
  void onGazeboClock(const ignition::msgs::Clock & message)
  {
    const auto & simulation_time = message.sim();
    const std::int64_t stamp_ns =
      static_cast<std::int64_t>(simulation_time.sec()) * 1000000000LL + simulation_time.nsec();
    if (stamp_ns < 0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring negative Gazebo simulation clock value.");
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    const bool reset = last_published_ns_ && stamp_ns < *last_published_ns_;
    const bool due =
      !last_published_ns_ || min_publish_interval_ns_ <= 0 || reset ||
      stamp_ns - *last_published_ns_ >= min_publish_interval_ns_;
    if (!due) {
      return;
    }

    rosgraph_msgs::msg::Clock output;
    output.clock.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
    output.clock.nanosec = static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
    clock_pub_->publish(output);
    last_published_ns_ = stamp_ns;
  }

  ignition::transport::Node transport_node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::mutex mutex_;
  std::optional<std::int64_t> last_published_ns_;
  std::int64_t min_publish_interval_ns_{0};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboClockRelay>());
  rclcpp::shutdown();
  return 0;
}
