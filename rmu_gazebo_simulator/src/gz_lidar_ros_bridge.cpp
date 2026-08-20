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

// Dedicated GZ-to-ROS LiDAR bridge for the active navigation profile. It keeps
// the existing PointCloud2 topic and message conversion, while isolating the
// high-rate PointCloudPacked callback from the generic multi-topic bridge.

#include <cstddef>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <ignition/msgs/pointcloud_packed.pb.h>
#include <ignition/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert/sensor_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace
{

class GazeboLidarRosBridge final : public rclcpp::Node
{
public:
  GazeboLidarRosBridge()
  : Node("gz_lidar_ros_bridge")
  {
    const auto gazebo_lidar_topic = declare_parameter<std::string>(
      "gazebo_lidar_topic",
      "/world/default/model/red_standard_robot1/link/front_mid360/sensor/"
      "front_mid360_lidar/scan/points");
    const auto ros_lidar_topic = declare_parameter<std::string>(
      "ros_lidar_topic", "/red_standard_robot1/livox/lidar");
    const auto publisher_depth = declare_parameter<std::int64_t>("publisher_depth", 1);
    if (gazebo_lidar_topic.empty() || ros_lidar_topic.empty() || publisher_depth <= 0) {
      throw std::invalid_argument(
              "gazebo_lidar_topic and ros_lidar_topic must be non-empty and publisher_depth positive");
    }

    rclcpp::QoS sensor_qos{rclcpp::KeepLast(static_cast<std::size_t>(publisher_depth))};
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();
    ros_lidar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(ros_lidar_topic, sensor_qos);

    if (!transport_node_.Subscribe(
        gazebo_lidar_topic, &GazeboLidarRosBridge::onGazeboLidar, this))
    {
      throw std::runtime_error(
              "Unable to subscribe to Gazebo LiDAR transport topic '" + gazebo_lidar_topic + "'");
    }
    worker_thread_ = std::thread(&GazeboLidarRosBridge::publishLatest, this);

    RCLCPP_INFO(
      get_logger(),
      "Direct Gazebo LiDAR bridge '%s' -> '%s' with BEST_EFFORT KeepLast(%ld).",
      gazebo_lidar_topic.c_str(), ros_lidar_topic.c_str(), publisher_depth);
  }

  ~GazeboLidarRosBridge() override
  {
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_shutdown_ = true;
    }
    worker_cv_.notify_one();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void onGazeboLidar(const ignition::msgs::PointCloudPacked & message)
  {
    // Gazebo Transport invokes this callback on a sensor delivery thread.
    // Copy into one replaceable slot so conversion and DDS publication cannot
    // stall source cadence; stale frames are intentionally discarded.
    auto latest = std::make_unique<ignition::msgs::PointCloudPacked>(message);
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      if (worker_shutdown_) {
        return;
      }
      latest_message_ = std::move(latest);
    }
    worker_cv_.notify_one();
  }

  void publishLatest()
  {
    for (;;) {
      std::unique_ptr<ignition::msgs::PointCloudPacked> latest;
      {
        std::unique_lock<std::mutex> lock(worker_mutex_);
        worker_cv_.wait(lock, [this]() {return worker_shutdown_ || latest_message_;});
        if (worker_shutdown_) {
          return;
        }
        latest = std::move(latest_message_);
      }

      sensor_msgs::msg::PointCloud2 output;
      ros_gz_bridge::convert_gz_to_ros(*latest, output);
      if (rclcpp::ok()) {
        ros_lidar_pub_->publish(output);
      }
    }
  }

  ignition::transport::Node transport_node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ros_lidar_pub_;
  std::mutex worker_mutex_;
  std::condition_variable worker_cv_;
  std::unique_ptr<ignition::msgs::PointCloudPacked> latest_message_;
  bool worker_shutdown_{false};
  std::thread worker_thread_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboLidarRosBridge>());
  rclcpp::shutdown();
  return 0;
}
