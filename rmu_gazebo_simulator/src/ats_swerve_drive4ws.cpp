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

// ATS-owned 4WD4WS Gazebo system plugin.
//
// Input: ignition.msgs.Twist in the chassis frame, [vx, vy, wz].
// Output: four physical steering-joint and wheel-joint velocity commands.
// The plugin also publishes Gazebo ground-truth odometry for regression only;
// that output must never substitute for the Point-LIO /localization chain.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/Conversions.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Pose.hh>

#include "rmu_gazebo_simulator/swerve_kinematics.hpp"

namespace ignition::gazebo::systems
{
namespace
{
constexpr int kModuleCount = 4;

struct SwerveModule
{
  std::string steer_joint_name;
  std::string wheel_joint_name;
  Entity steer_joint{kNullEntity};
  Entity wheel_joint{kNullEntity};
  double x{0.0};
  double y{0.0};
  double target_steer_angle{0.0};
};
}  // namespace

class AtsSwerveDrive4WSPrivate
{
public:
  void OnCmdVel(const ignition::msgs::Twist & message)
  {
    std::lock_guard<std::mutex> lock(target_velocity_mutex);
    target_velocity = message;
    last_command_time = std::chrono::steady_clock::now();
    received_command = true;
  }

  void PublishGroundTruth(
    const UpdateInfo & info, const EntityComponentManager & ecm)
  {
    const auto * pose = ecm.Component<components::WorldPose>(chassis_link);
    const auto * linear = ecm.Component<components::LinearVelocity>(chassis_link);
    const auto * angular = ecm.Component<components::AngularVelocity>(chassis_link);
    if (pose == nullptr || linear == nullptr || angular == nullptr) {
      return;
    }

    const auto chassis_pose = pose->Data();
    const auto linear_velocity = linear->Data();
    const auto angular_velocity = angular->Data();

    msgs::Odometry message;
    message.mutable_pose()->mutable_position()->set_x(chassis_pose.X());
    message.mutable_pose()->mutable_position()->set_y(chassis_pose.Y());
    message.mutable_pose()->mutable_position()->set_z(chassis_pose.Z());
    msgs::Set(message.mutable_pose()->mutable_orientation(), chassis_pose.Rot());
    message.mutable_twist()->mutable_linear()->set_x(linear_velocity.X());
    message.mutable_twist()->mutable_linear()->set_y(linear_velocity.Y());
    message.mutable_twist()->mutable_linear()->set_z(linear_velocity.Z());
    message.mutable_twist()->mutable_angular()->set_x(angular_velocity.X());
    message.mutable_twist()->mutable_angular()->set_y(angular_velocity.Y());
    message.mutable_twist()->mutable_angular()->set_z(angular_velocity.Z());
    message.mutable_header()->mutable_stamp()->CopyFrom(convert<msgs::Time>(info.simTime));

    auto frame = message.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(odom_frame_id);
    auto child_frame = message.mutable_header()->add_data();
    child_frame->set_key("child_frame_id");
    child_frame->add_value(odom_child_frame_id);
    odometry_publisher.Publish(message);
  }

  transport::Node transport_node;
  Model model{kNullEntity};
  std::string chassis_link_name;
  Entity chassis_link{kNullEntity};
  std::vector<SwerveModule> modules;
  double wheel_radius{0.0425};
  double max_wheel_speed{39.26991};
  double max_steer_rate{12.0};
  double steer_p_gain{12.0};
  double command_timeout{0.5};
  bool received_command{false};
  msgs::Twist target_velocity;
  std::chrono::steady_clock::time_point last_command_time;
  std::mutex target_velocity_mutex;
  std::string odom_frame_id;
  std::string odom_child_frame_id;
  transport::Node::Publisher odometry_publisher;
};

class AtsSwerveDrive4WS
  : public System,
    public ISystemConfigure,
    public ISystemPreUpdate,
    public ISystemPostUpdate
{
public:
  AtsSwerveDrive4WS()
  : data_(std::make_unique<AtsSwerveDrive4WSPrivate>())
  {
  }

  ~AtsSwerveDrive4WS() override = default;

  void Configure(
    const Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    EntityComponentManager & ecm, EventManager &) override
  {
    data_->model = Model(entity);
    if (!data_->model.Valid(ecm)) {
      ignerr << "AtsSwerveDrive4WS must be attached to a model." << std::endl;
      return;
    }

    data_->chassis_link_name = sdf->Get<std::string>("chassis_link", "chassis").first;
    data_->chassis_link = data_->model.LinkByName(ecm, data_->chassis_link_name);
    if (data_->chassis_link == kNullEntity) {
      ignerr << "AtsSwerveDrive4WS chassis link [" << data_->chassis_link_name
             << "] was not found." << std::endl;
      return;
    }

    data_->wheel_radius = sdf->Get<double>("wheel_radius", 0.0425).first;
    data_->max_wheel_speed = sdf->Get<double>("max_wheel_speed", 39.26991).first;
    data_->max_steer_rate = sdf->Get<double>("max_steer_rate", 12.0).first;
    data_->steer_p_gain = sdf->Get<double>("steer_p_gain", 12.0).first;
    data_->command_timeout = sdf->Get<double>("command_timeout", 0.5).first;
    if (data_->wheel_radius <= 0.0) {
      ignerr << "AtsSwerveDrive4WS wheel_radius must be positive." << std::endl;
      return;
    }

    auto sdf_clone = sdf->Clone();
    for (auto module_element = sdf_clone->GetElement("module"); module_element;
      module_element = module_element->GetNextElement("module"))
    {
      SwerveModule module;
      module.steer_joint_name = module_element->Get<std::string>("steer_joint", "").first;
      module.wheel_joint_name = module_element->Get<std::string>("wheel_joint", "").first;
      module.x = module_element->Get<double>("x", 0.0).first;
      module.y = module_element->Get<double>("y", 0.0).first;
      module.steer_joint = data_->model.JointByName(ecm, module.steer_joint_name);
      module.wheel_joint = data_->model.JointByName(ecm, module.wheel_joint_name);
      if (module.steer_joint == kNullEntity || module.wheel_joint == kNullEntity) {
        ignerr << "AtsSwerveDrive4WS module joints were not found: steer=["
               << module.steer_joint_name << "], wheel=[" << module.wheel_joint_name
               << "]." << std::endl;
        data_->modules.clear();
        return;
      }
      data_->modules.push_back(module);
    }
    if (static_cast<int>(data_->modules.size()) != kModuleCount) {
      ignerr << "AtsSwerveDrive4WS requires exactly four <module> elements, got ["
             << data_->modules.size() << "]." << std::endl;
      data_->modules.clear();
      return;
    }

    const std::string command_topic = sdf->Get<std::string>(
      "topic", data_->model.Name(ecm) + "/cmd_vel").first;
    data_->transport_node.Subscribe(
      command_topic, &AtsSwerveDrive4WSPrivate::OnCmdVel, data_.get());
    ignmsg << "AtsSwerveDrive4WS subscribing on [" << command_topic << "]" << std::endl;

    const auto model_name = data_->model.Name(ecm);
    data_->odometry_publisher = data_->transport_node.Advertise<msgs::Odometry>(
      model_name + "/odometry");
    data_->odom_frame_id = model_name + "/odom";
    data_->odom_child_frame_id = model_name + "/" +
      ignition::common::replaceAll(data_->chassis_link_name, "::", "/");
  }

  void PreUpdate(const UpdateInfo & info, EntityComponentManager & ecm) override
  {
    IGN_PROFILE("AtsSwerveDrive4WS::PreUpdate");
    if (info.paused || data_->modules.empty()) {
      return;
    }

    if (!ecm.Component<components::WorldPose>(data_->chassis_link)) {
      ecm.CreateComponent(data_->chassis_link, components::WorldPose());
    }
    if (!ecm.Component<components::LinearVelocity>(data_->chassis_link)) {
      ecm.CreateComponent(data_->chassis_link, components::LinearVelocity());
    }
    if (!ecm.Component<components::AngularVelocity>(data_->chassis_link)) {
      ecm.CreateComponent(data_->chassis_link, components::AngularVelocity());
    }
    for (const auto & module : data_->modules) {
      if (!ecm.Component<components::JointPosition>(module.steer_joint)) {
        ecm.CreateComponent(module.steer_joint, components::JointPosition());
      }
      if (!ecm.Component<components::JointVelocity>(module.wheel_joint)) {
        ecm.CreateComponent(module.wheel_joint, components::JointVelocity());
      }
    }

    msgs::Twist target;
    bool stale = false;
    {
      std::lock_guard<std::mutex> lock(data_->target_velocity_mutex);
      target = data_->target_velocity;
      if (data_->command_timeout > 0.0) {
        stale = !data_->received_command ||
          std::chrono::duration<double>(
          std::chrono::steady_clock::now() - data_->last_command_time).count() >
          data_->command_timeout;
      }
    }

    double vx = stale ? 0.0 : target.linear().x();
    double vy = stale ? 0.0 : target.linear().y();
    double wz = stale ? 0.0 : target.angular().z();
    const bool zero_command =
      std::abs(vx) < 1e-6 && std::abs(vy) < 1e-6 && std::abs(wz) < 1e-6;

    for (auto & module : data_->modules) {
      double measured_angle = module.target_steer_angle;
      if (const auto * position = ecm.Component<components::JointPosition>(module.steer_joint);
        position != nullptr && !position->Data().empty())
      {
        measured_angle = position->Data()[0];
      }

      const auto setpoint = rmu_gazebo_simulator::swerve::ComputeModuleSetpoint(
        {vx, vy, wz}, {module.x, module.y}, data_->wheel_radius,
        measured_angle, module.target_steer_angle);
      module.target_steer_angle = setpoint.steering_angle;
      const double angle_error = rmu_gazebo_simulator::swerve::WrapPi(
        module.target_steer_angle - measured_angle);
      double wheel_speed = setpoint.wheel_angular_velocity;

      double steer_rate = std::clamp(
        data_->steer_p_gain * angle_error, -data_->max_steer_rate, data_->max_steer_rate);
      wheel_speed = std::clamp(
        wheel_speed, -data_->max_wheel_speed, data_->max_wheel_speed);
      if (zero_command) {
        steer_rate = 0.0;
        wheel_speed = 0.0;
      }

      auto * steer_command = ecm.Component<components::JointVelocityCmd>(module.steer_joint);
      if (steer_command == nullptr) {
        ecm.CreateComponent(module.steer_joint, components::JointVelocityCmd({steer_rate}));
      } else {
        *steer_command = components::JointVelocityCmd({steer_rate});
      }
      auto * wheel_command = ecm.Component<components::JointVelocityCmd>(module.wheel_joint);
      if (wheel_command == nullptr) {
        ecm.CreateComponent(module.wheel_joint, components::JointVelocityCmd({wheel_speed}));
      } else {
        *wheel_command = components::JointVelocityCmd({wheel_speed});
      }
    }
  }

  void PostUpdate(const UpdateInfo & info, const EntityComponentManager & ecm) override
  {
    if (!info.paused && !data_->modules.empty()) {
      data_->PublishGroundTruth(info, ecm);
    }
  }

private:
  std::unique_ptr<AtsSwerveDrive4WSPrivate> data_;
};

}  // namespace ignition::gazebo::systems

IGNITION_ADD_PLUGIN(
  ignition::gazebo::systems::AtsSwerveDrive4WS,
  ignition::gazebo::System,
  ignition::gazebo::systems::AtsSwerveDrive4WS::ISystemConfigure,
  ignition::gazebo::systems::AtsSwerveDrive4WS::ISystemPreUpdate,
  ignition::gazebo::systems::AtsSwerveDrive4WS::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(
  ignition::gazebo::systems::AtsSwerveDrive4WS,
  "ignition::gazebo::systems::AtsSwerveDrive4WS")
