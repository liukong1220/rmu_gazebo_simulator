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

/// \file
/// Adapt the Gazebo mid360 sensor stream to the Point-LIO input contract.
///
/// Point-LIO is configured with ``preprocess.lidar_type: 1``, so it subscribes
/// to ``livox_ros_driver2/msg/CustomMsg`` and not to
/// ``sensor_msgs/msg/PointCloud2``. The Gazebo gpu_lidar sensor can only emit
/// PointCloud2, so without this node the simulated LiDAR never reaches the
/// localization chain at all.
///
/// The node is a pure format/frame adapter:
///
/// * it does not synthesise poses, so it cannot stand in for localization;
/// * it does not publish ``/localization`` or ``/registered_scan`` itself -
///   those stay owned by Point-LIO -> loam_interface ->
///   sensor_scan_generation;
/// * it keeps the sensor timestamps untouched so ``use_sim_time`` freshness and
///   stale-lease checks downstream keep their meaning.
///
/// This is C++ rather than Python for a measured reason. The rclpy version of
/// this adapter added a fixed 0.204 s to every frame, which pushed the
/// downstream ``/odometry`` and ``/localization`` stamps roughly 0.33 s behind
/// ``/clock`` and made the planning-grid adapter's ``gimbal_yaw_odom -> map``
/// lookup fail with "extrapolation into the future" on TF stamps that sit on a
/// 0.1 s grid. Profiling put that cost in the per-point rosidl Python object
/// allocation: 18430 ``CustomPoint`` instances cost 120.8 ms, a plain
/// ``__slots__`` stand-in cost 37.5 ms, and assigning the finished list to
/// ``msg.points`` cost 1.8 ms. Vectorising the filter with numpy took the
/// filtering itself from 112.3 ms to 25.2 ms but could not touch the
/// allocation, and ``PYTHONOPTIMIZE=1`` (which strips the rosidl setter type
/// assertions) only recovered a quarter of it, 126.5 ms to 95.3 ms. The
/// remaining cost is inherent to building the message in Python.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {

rclcpp::QoS sensor_qos(size_t depth) {
  // Braces, not parentheses: `QoS qos(rclcpp::KeepLast(depth))` is a function
  // declaration, not a variable, and the compiler then rejects every member
  // access below.
  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  qos.best_effort();
  qos.durability_volatile();
  return qos;
}

} // namespace

class GzLivoxBridge : public rclcpp::Node {
public:
  GzLivoxBridge() : Node("gz_livox_bridge") {
    const std::string input_cloud_topic = this->declare_parameter<std::string>(
        "input_cloud_topic", "/red_standard_robot1/livox/lidar");
    const std::string input_imu_topic = this->declare_parameter<std::string>(
        "input_imu_topic", "/red_standard_robot1/livox/imu");
    const std::string output_cloud_topic = this->declare_parameter<std::string>(
        "output_cloud_topic", "/livox/lidar");
    const std::string output_imu_topic =
        this->declare_parameter<std::string>("output_imu_topic", "/livox/imu");
    // Point-LIO resolves every LiDAR point in this frame; it must match
    // `sensor_frame`/`lidar_frame` used by the ATS chain (front_mid360).
    lidar_frame_id_ =
        this->declare_parameter<std::string>("lidar_frame_id", "front_mid360");
    imu_frame_id_ =
        this->declare_parameter<std::string>("imu_frame_id", "front_mid360");
    // Gazebo's gpu_lidar has no per-point timestamps, so points inside one
    // scan are spread uniformly over this window instead of being faked as
    // simultaneous. 0 keeps every offset_time at zero.
    // The ATS Mid360 SDF emits 10 complete frames per second.  Point-LIO
    // consumes the same 0.1 s frame interval, so its synthetic per-point
    // offsets must not describe an impossible 20 Hz sensor.
    scan_period_sec_ = this->declare_parameter<double>("scan_period_sec", 0.1);
    min_range_ = this->declare_parameter<double>("min_range", 0.3);
    max_range_ = this->declare_parameter<double>("max_range", 40.0);

    cloud_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
        output_cloud_topic, sensor_qos(5));
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(output_imu_topic,
                                                             sensor_qos(20));

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_cloud_topic, sensor_qos(5),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(msg);
        });
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        input_imu_topic, sensor_qos(20),
        [this](sensor_msgs::msg::Imu::UniquePtr msg) {
          imuCallback(std::move(msg));
        });

    RCLCPP_INFO(
        this->get_logger(),
        "gz_livox_bridge: cloud '%s' -> '%s', imu '%s' -> '%s', frame '%s'",
        input_cloud_topic.c_str(), output_cloud_topic.c_str(),
        input_imu_topic.c_str(), output_imu_topic.c_str(),
        lidar_frame_id_.c_str());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    // Non-finite points are dropped before anything is indexed, so that
    // offset_time spreads the *kept-by-finiteness* points evenly over the scan
    // window. Range rejection happens afterwards and deliberately does not
    // renumber: a point's offset_time reflects when the sensor swept past it,
    // not its position in the filtered output.
    struct InputPoint {
      float x;
      float y;
      float z;
      std::uint8_t line;
    };
    std::vector<InputPoint> finite;
    finite.reserve(static_cast<size_t>(msg->width) *
                   static_cast<size_t>(msg->height));

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      std::size_t source_index = 0;
      for (; iter_x != iter_x.end();
           ++iter_x, ++iter_y, ++iter_z, ++source_index) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        // Gazebo lays this organised cloud out as height vertical rows by
        // width horizontal samples. Preserve the Mid360 ring rather than
        // labelling every point as line 0, otherwise Point-LIO cannot apply
        // its configured 32-line preprocessing contract.
        const std::uint8_t line =
            msg->height > 1U && msg->width > 0U
                ? static_cast<std::uint8_t>(std::min<std::size_t>(
                      source_index / static_cast<std::size_t>(msg->width),
                      255U))
                : 0U;
        finite.push_back({x, y, z, line});
      }
    } catch (const std::runtime_error &error) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Dropping cloud without float32 x/y/z fields: %s",
                           error.what());
      return;
    }

    const size_t total = finite.size();
    const int64_t span_ns = static_cast<int64_t>(scan_period_sec_ * 1e9);

    auto out = std::make_unique<livox_ros_driver2::msg::CustomMsg>();
    out->header = msg->header;
    out->header.frame_id = lidar_frame_id_;
    out->timebase =
        static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
        static_cast<uint64_t>(msg->header.stamp.nanosec);
    out->lidar_id = 0;
    out->points.reserve(total);

    for (size_t index = 0; index < total; ++index) {
      const double x = static_cast<double>(finite[index].x);
      const double y = static_cast<double>(finite[index].y);
      const double z = static_cast<double>(finite[index].z);
      const double distance = std::sqrt(x * x + y * y + z * z);
      if (distance < min_range_ || distance > max_range_) {
        continue;
      }
      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time =
          span_ns > 0
              ? static_cast<uint32_t>(span_ns * static_cast<int64_t>(index) /
                                      static_cast<int64_t>(total))
              : 0U;
      point.x = finite[index].x;
      point.y = finite[index].y;
      point.z = finite[index].z;
      point.reflectivity = 100;
      point.tag = 0;
      point.line = finite[index].line;
      out->points.push_back(point);
    }

    out->point_num = static_cast<uint32_t>(out->points.size());
    cloud_pub_->publish(std::move(out));
  }

  void imuCallback(sensor_msgs::msg::Imu::UniquePtr msg) {
    msg->header.frame_id = imu_frame_id_;
    imu_pub_->publish(std::move(msg));
  }

  std::string lidar_frame_id_;
  std::string imu_frame_id_;
  double scan_period_sec_{0.05};
  double min_range_{0.3};
  double max_range_{40.0};

  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzLivoxBridge>());
  rclcpp::shutdown();
  return 0;
}
