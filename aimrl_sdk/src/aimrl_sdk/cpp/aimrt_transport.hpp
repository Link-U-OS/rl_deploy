#pragma once

#include "core.hpp"

#include <filesystem>
#include <future>
#include <stdexcept>
#include <utility>

#include "aimrt_module_cpp_interface/core.h"
#include "aimrt_module_ros2_interface/channel/ros2_channel.h"
#include "core/aimrt_core.h"

#include "joint_msgs/msg/joint_command.hpp"
#include "joint_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace aimrl_sdk {

class AimrtTransport final : public Transport {
 public:
  AimrtTransport(const std::string &cfg_file_path) {
    options_.cfg_file_path = cfg_file_path;
  }

  ~AimrtTransport() override {
    try {
      stop();
    } catch (...) {
    }
  }

  void start(Callbacks callbacks) override {
    if (started_) {
      return;
    }
    started_ = true;

    callbacks_ = std::move(callbacks);

    if (!std::filesystem::exists(options_.cfg_file_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("LeggedSystemHardware"), "Config file not found: %s", options_.cfg_file_path.c_str());
      exit(-1);
    }

    aimrt_core_.Initialize(options_);

    // Create Module
    module_handle_ = aimrt::CoreRef(aimrt_core_.GetModuleManager().CreateModule("LeggedSystemModule"));

    // Create Publishers
    aimrtLegCmdPublisher_ = module_handle_.GetChannelHandle().GetPublisher("/body_drive/leg_joint_command");
    aimrtArmCmdPublisher_ = module_handle_.GetChannelHandle().GetPublisher("/body_drive/arm_joint_command");
    bool ret = aimrt::channel::RegisterPublishType<joint_msgs::msg::JointCommand>(aimrtLegCmdPublisher_);
    if (!ret) {
      throw std::runtime_error("Failed to register publish type for topic '/body_drive/leg_joint_command'");
    }
    ret = aimrt::channel::RegisterPublishType<joint_msgs::msg::JointCommand>(aimrtArmCmdPublisher_);
    if (!ret) {
      throw std::runtime_error("Failed to register publish type for topic '/body_drive/arm_joint_command'");
    }
    aimrtLegCmdPublisherProxy_ = std::make_unique<aimrt::channel::PublisherProxy<joint_msgs::msg::JointCommand>>(aimrtLegCmdPublisher_);
    aimrtArmCmdPublisherProxy_ = std::make_unique<aimrt::channel::PublisherProxy<joint_msgs::msg::JointCommand>>(aimrtArmCmdPublisher_);

    // Create Subscribers
    aimrtArmStateSubscriber_ = module_handle_.GetChannelHandle().GetSubscriber("/body_drive/arm_joint_state");
    aimrtLegStateSubscriber_ = module_handle_.GetChannelHandle().GetSubscriber("/body_drive/leg_joint_state");
    aimrtImuSubscriber_ = module_handle_.GetChannelHandle().GetSubscriber("/body_drive/imu/data");

    ret = aimrt::channel::Subscribe<joint_msgs::msg::JointState>(aimrtArmStateSubscriber_, std::move(callbacks_.on_arm_state));
    if (!ret) {
      throw std::runtime_error("Failed to subscribe to topic '/body_drive/arm_joint_state'");
    }
    ret = aimrt::channel::Subscribe<joint_msgs::msg::JointState>(aimrtLegStateSubscriber_, std::move(callbacks_.on_leg_state));
    if (!ret) {
      throw std::runtime_error("Failed to subscribe to topic '/body_drive/leg_joint_state'");
    }
    ret = aimrt::channel::Subscribe<sensor_msgs::msg::Imu>(aimrtImuSubscriber_, std::move(callbacks_.on_imu));
    if (!ret) {
      throw std::runtime_error("Failed to subscribe to topic '/body_drive/imu/data'");
    }
    shutdown_future_ = aimrt_core_.AsyncStart();
  }

  void stop() override {
    if (!started_) {
      return;
    }
    started_ = false;

    try {
      aimrt_core_.Shutdown();
    } catch (...) {
    }

    // Block until AsyncStart's internal shutdown thread finishes so that
    // AimRTCore's destructor can't run while shutdown is still in progress.
    if (shutdown_future_.valid()) {
      try {
        shutdown_future_.get();
      } catch (...) {
      }
    }
  }

  void publish_arm_command(TimestampNs stamp, Sequence32 seq,
                           const PendingCommand<kArmDof> &cmd,
                           std::span<const std::string> arm_names) override {
    if (!aimrtArmCmdPublisherProxy_ || !cmd.has_any)
      return;
    const auto arm_dof = static_cast<std::size_t>(kArmDof);
    if (arm_names.size() != arm_dof)
      throw std::invalid_argument("arm_names size mismatch");

    joint_msgs::msg::JointCommand msg;
    const auto stamp_ns = stamp.value;
    msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(stamp_ns % 1000000000LL);

    msg.joints.resize(arm_dof);
    for (std::size_t i = 0; i < arm_dof; ++i) {
      auto &joint = msg.joints[i];
      joint.name = arm_names[i];
      joint.sequence = seq.value;
      joint.position = cmd.pos[i];
      joint.velocity = cmd.vel[i];
      joint.effort = cmd.eff[i];
      joint.stiffness = cmd.kp[i];
      joint.damping = cmd.kd[i];
    }

    aimrtArmCmdPublisherProxy_->Publish(msg);
  }
  void publish_leg_command(TimestampNs stamp, Sequence32 seq,
                           const PendingCommand<kLegDof> &cmd,
                           std::span<const std::string> leg_names) override {
    if (!aimrtLegCmdPublisherProxy_ || !cmd.has_any)
      return;
    const auto leg_dof = static_cast<std::size_t>(kLegDof);
    if (leg_names.size() != leg_dof)
      throw std::invalid_argument("leg_names size mismatch");

    joint_msgs::msg::JointCommand msg;
    const auto stamp_ns = stamp.value;
    msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(stamp_ns % 1000000000LL);

    msg.joints.resize(leg_dof);
    for (std::size_t i = 0; i < leg_dof; ++i) {
      auto &joint = msg.joints[i];
      joint.name = leg_names[i];
      joint.sequence = seq.value;
      joint.position = cmd.pos[i];
      joint.velocity = cmd.vel[i];
      joint.effort = cmd.eff[i];
      joint.stiffness = cmd.kp[i];
      joint.damping = cmd.kd[i];
    }

    aimrtLegCmdPublisherProxy_->Publish(msg);
  }

 private:
  bool started_{false};
  std::future<void> shutdown_future_;

  Callbacks callbacks_{};
  aimrt::runtime::core::AimRTCore aimrt_core_;
  aimrt::runtime::core::AimRTCore::Options options_;
  aimrt::CoreRef module_handle_;
  aimrt::channel::SubscriberRef aimrtArmStateSubscriber_;
  aimrt::channel::SubscriberRef aimrtLegStateSubscriber_;
  aimrt::channel::SubscriberRef aimrtImuSubscriber_;
  aimrt::channel::PublisherRef aimrtLegCmdPublisher_;
  aimrt::channel::PublisherRef aimrtArmCmdPublisher_;
  std::unique_ptr<aimrt::channel::PublisherProxy<joint_msgs::msg::JointCommand>> aimrtLegCmdPublisherProxy_;
  std::unique_ptr<aimrt::channel::PublisherProxy<joint_msgs::msg::JointCommand>> aimrtArmCmdPublisherProxy_;
};

}  // namespace aimrl_sdk
