// Copyright 2024 TIER IV, Inc. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include "autoware/rtc_interface/rtc_interface.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::rtc_interface
{
class RTCInterfaceTest : public ::testing::Test
{
public:
  bool callCooperate(CooperateCommands::Request & request, CooperateCommands::Response & response)
  {
    return rtc_interface_->onCooperateCommandService(request, response);
  }

  bool callAutoMode(AutoMode::Request & request, AutoMode::Response & response)
  {
    return rtc_interface_->onAutoModeService(request, response);
  }

  const std::vector<CooperateCommand> & storedCommands() const
  {
    return rtc_interface_->stored_commands_;
  }

  const CooperateStatusArray & registeredStatuses() const
  {
    return rtc_interface_->registered_status_;
  }

  // Mechanical ROS1 counterparts of the upstream friend-fixture accessors.
  void update_cooperate_command_status(const std::vector<CooperateCommand> & commands)
  {
    rtc_interface_->updateCooperateCommandStatus(commands);
  }

  std::vector<CooperateCommand> get_stored_command() const
  {
    return rtc_interface_->stored_commands_;
  }

protected:
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<RTCInterface> rtc_interface_;

  static UUID uuid(const uint8_t value)
  {
    UUID result;
    result.uuid.fill(value);
    return result;
  }

  static CooperateCommand command(const UUID & id, const uint8_t type)
  {
    CooperateCommand result;
    result.uuid = id;
    result.module.type = Module::INTERSECTION;
    result.command.type = type;
    return result;
  }

  void registerStatus(const UUID & id, const uint8_t state = State::WAITING_FOR_EXECUTION)
  {
    rtc_interface_->updateCooperateStatus(
      id, true, State::WAITING_FOR_EXECUTION, 10.0, 100.0, ros::Time(1, 0));
    if (state == State::SUCCEEDED) {
      rtc_interface_->updateCooperateStatus(id, true, State::RUNNING, 10.0, 100.0, ros::Time(2, 0));
      rtc_interface_->updateCooperateStatus(id, true, state, 10.0, 100.0, ros::Time(3, 0));
    } else if (state != State::WAITING_FOR_EXECUTION) {
      rtc_interface_->updateCooperateStatus(id, true, state, 10.0, 100.0, ros::Time(2, 0));
    }
  }

  void SetUp() override
  {
    if (!ros::isInitialized()) {
      int argc = 0;
      char ** argv = nullptr;
      ros::init(argc, argv, "rtc_interface_p2_test", ros::init_options::AnonymousName);
    }
    node_ = std::make_unique<ros::NodeHandle>();
    rtc_interface_ = std::make_unique<RTCInterface>(node_.get(), "intersection", false);
  }

  void TearDown() override
  {
    rtc_interface_.reset();
    node_.reset();
  }
};

TEST_F(RTCInterfaceTest, rtc_service_valid_waiting_transport_true_and_payload)
{
  const auto id = uuid(0x11);
  registerStatus(id);
  CooperateCommands::Request request;
  request.commands = {command(id, Command::ACTIVATE)};
  CooperateCommands::Response response;

  EXPECT_TRUE(callCooperate(request, response));
  ASSERT_EQ(response.responses.size(), 1U);
  EXPECT_EQ(response.responses.front().uuid, id);
  EXPECT_EQ(response.responses.front().module.type, Module::INTERSECTION);
  EXPECT_TRUE(response.responses.front().success);
  ASSERT_EQ(registeredStatuses().statuses.size(), 1U);
  EXPECT_EQ(registeredStatuses().statuses.front().command_status.type, Command::ACTIVATE);
  EXPECT_FALSE(registeredStatuses().statuses.front().auto_mode);
}

TEST_F(RTCInterfaceTest, rtc_service_terminal_and_missing_transport_true_payload_false)
{
  const auto terminal = uuid(0x22);
  const auto missing = uuid(0x23);
  registerStatus(terminal, State::SUCCEEDED);
  CooperateCommands::Request request;
  request.commands = {command(terminal, Command::ACTIVATE), command(missing, Command::DEACTIVATE)};
  CooperateCommands::Response response;

  EXPECT_TRUE(callCooperate(request, response));
  ASSERT_EQ(response.responses.size(), 2U);
  EXPECT_FALSE(response.responses[0].success);
  EXPECT_FALSE(response.responses[1].success);
  EXPECT_FALSE(rtc_interface_->isRegistered(missing));
}

TEST_F(RTCInterfaceTest, rtc_service_ignores_stamp_and_preserves_response_order_uuid_module)
{
  const auto first = uuid(0x31);
  const auto second = uuid(0x32);
  registerStatus(first);
  registerStatus(second, State::FAILED);
  CooperateCommands::Request request;
  request.stamp = ros::Time(999, 123);
  request.commands = {command(second, Command::ACTIVATE), command(first, Command::DEACTIVATE)};
  CooperateCommands::Response response;

  EXPECT_TRUE(callCooperate(request, response));
  ASSERT_EQ(response.responses.size(), 2U);
  EXPECT_EQ(response.responses[0].uuid, second);
  EXPECT_EQ(response.responses[0].module.type, Module::INTERSECTION);
  EXPECT_FALSE(response.responses[0].success);
  EXPECT_EQ(response.responses[1].uuid, first);
  EXPECT_EQ(response.responses[1].module.type, Module::INTERSECTION);
  EXPECT_TRUE(response.responses[1].success);
  EXPECT_TRUE(storedCommands().empty());
}

TEST_F(RTCInterfaceTest, rtc_service_locked_replaces_stored_commands_then_unlock_applies_once)
{
  const auto id = uuid(0x41);
  registerStatus(id);
  ASSERT_EQ(registeredStatuses().statuses.size(), 1U);
  EXPECT_EQ(registeredStatuses().statuses.front().command_status.type, Command::DEACTIVATE);
  EXPECT_TRUE(registeredStatuses().statuses.front().auto_mode);
  rtc_interface_->lockCommandUpdate();
  CooperateCommands::Request first;
  first.commands = {command(id, Command::DEACTIVATE)};
  CooperateCommands::Request second;
  second.commands = {command(id, Command::ACTIVATE)};
  CooperateCommands::Response first_response;
  CooperateCommands::Response second_response;

  EXPECT_TRUE(callCooperate(first, first_response));
  EXPECT_TRUE(callCooperate(second, second_response));
  ASSERT_EQ(first_response.responses.size(), 1U);
  ASSERT_EQ(second_response.responses.size(), 1U);
  EXPECT_TRUE(first_response.responses.front().success);
  EXPECT_TRUE(second_response.responses.front().success);
  ASSERT_EQ(storedCommands().size(), 1U);
  EXPECT_EQ(storedCommands().front().command.type, Command::ACTIVATE);
  // Locking defers every valid command mutation; the later request only replaces storage.
  EXPECT_EQ(registeredStatuses().statuses.front().command_status.type, Command::DEACTIVATE);
  EXPECT_TRUE(registeredStatuses().statuses.front().auto_mode);
  rtc_interface_->unlockCommandUpdate();
  EXPECT_EQ(registeredStatuses().statuses.front().command_status.type, Command::ACTIVATE);
  EXPECT_FALSE(registeredStatuses().statuses.front().auto_mode);
}

TEST_F(RTCInterfaceTest, rtc_automode_true_then_false_broadcast)
{
  const auto first = uuid(0x51);
  const auto second = uuid(0x52);
  registerStatus(first);
  registerStatus(second);
  for (const bool enabled : {true, false}) {
    AutoMode::Request request;
    request.enable = enabled;
    AutoMode::Response response;
    EXPECT_TRUE(callAutoMode(request, response));
    EXPECT_TRUE(response.success);
    for (const auto & status : registeredStatuses().statuses) {
      EXPECT_EQ(status.auto_mode, enabled);
    }
  }
}

TEST_F(RTCInterfaceTest, rtc_service_overlap_has_no_private_spinner_and_no_forced_order_claim)
{
  const auto id = uuid(0x61);
  registerStatus(id);
  CooperateCommands::Request commands;
  commands.commands = {command(id, Command::ACTIVATE)};
  AutoMode::Request automode;
  automode.enable = false;
  CooperateCommands::Response command_response;
  AutoMode::Response automode_response;
  std::atomic<bool> command_transport{false};
  std::atomic<bool> automode_transport{false};
  std::atomic<bool> status_mutation_completed{false};

  std::thread command_thread([&] {
    command_transport = callCooperate(commands, command_response);
  });
  std::thread automode_thread([&] {
    automode_transport = callAutoMode(automode, automode_response);
  });
  std::thread status_mutation_thread([&] {
    rtc_interface_->updateCooperateStatus(
      id, true, State::RUNNING, 10.0, 100.0, ros::Time(2, 0));
    status_mutation_completed = true;
  });
  command_thread.join();
  automode_thread.join();
  status_mutation_thread.join();

  EXPECT_TRUE(command_transport);
  EXPECT_TRUE(automode_transport);
  EXPECT_TRUE(status_mutation_completed);
  ASSERT_EQ(command_response.responses.size(), 1U);
  EXPECT_EQ(command_response.responses.front().uuid, id);
  EXPECT_TRUE(command_response.responses.front().success);
  EXPECT_TRUE(automode_response.success);
  // The mutex serializes state operations; no completion order is asserted.
  ASSERT_EQ(registeredStatuses().statuses.size(), 1U);
  EXPECT_EQ(registeredStatuses().statuses.front().uuid, id);
  EXPECT_EQ(registeredStatuses().statuses.front().state.type, State::RUNNING);
  EXPECT_EQ(registeredStatuses().statuses.front().command_status.type, Command::ACTIVATE);
  EXPECT_FALSE(registeredStatuses().statuses.front().auto_mode);
}

TEST_F(RTCInterfaceTest, uuid_to_string)
{
  // Source assertion mapping: upstream test_rtc_interface.cpp:57-176.
  auto id = autoware_utils::generate_uuid();
  ros::Time stamp(1, 0);

  // Source lines 62-68: no registered UUID.
  EXPECT_FALSE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_TRUE(rtc_interface_->isTerminated(id));

  // Source lines 70-78: WAITING_FOR_EXECUTION registration.
  rtc_interface_->updateCooperateStatus(
    id, true, State::WAITING_FOR_EXECUTION, 10.0, 100.0, stamp);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_TRUE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_FALSE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 80-82: removal.
  rtc_interface_->removeCooperateStatus(id);
  EXPECT_FALSE(rtc_interface_->isRegistered(id));

  // Source lines 84-91: invalid initial FAILED remains WAITING.
  rtc_interface_->updateCooperateStatus(id, true, State::FAILED, 10.0, 100.0, stamp);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_TRUE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_FALSE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 93-106: force activation.
  std::vector<CooperateCommand> commands;
  CooperateCommand command;
  command.uuid = id;
  command.command.type = Command::ACTIVATE;
  commands.push_back(command);
  rtc_interface_->updateCooperateStatus(id, false, State::RUNNING, 10.0, 100.0, stamp);
  update_cooperate_command_status(commands);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_TRUE(rtc_interface_->isActivated(id));
  EXPECT_TRUE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 108-115: SUCCEEDED termination.
  rtc_interface_->updateCooperateStatus(id, false, State::SUCCEEDED, 10.0, 100.0, stamp);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_TRUE(rtc_interface_->isTerminated(id));

  // Source lines 117-120: clear state and stored commands.
  rtc_interface_->clearCooperateStatus();
  EXPECT_FALSE(rtc_interface_->isRegistered(id));
  EXPECT_TRUE(get_stored_command().empty());

  // Source lines 122-133: force deactivation.
  commands.front().command.type = Command::DEACTIVATE;
  rtc_interface_->updateCooperateStatus(
    id, true, State::WAITING_FOR_EXECUTION, 10.0, 100.0, stamp);
  rtc_interface_->updateCooperateStatus(id, true, State::RUNNING, 10.0, 100.0, stamp);
  update_cooperate_command_status(commands);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_TRUE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 135-142: ABORTING transition.
  rtc_interface_->updateCooperateStatus(id, true, State::ABORTING, 10.0, 100.0, stamp);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 144-151: FAILED termination.
  rtc_interface_->updateCooperateStatus(id, true, State::FAILED, 10.0, 100.0, stamp);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_TRUE(rtc_interface_->isTerminated(id));

  rtc_interface_->clearCooperateStatus();

  // Source lines 155-165: requested state and RUNNING transition.
  rtc_interface_->updateCooperateStatus(
    id, true, State::WAITING_FOR_EXECUTION, 10.0, 100.0, stamp, true);
  rtc_interface_->updateCooperateStatus(id, true, State::RUNNING, 10.0, 100.0, stamp, true);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_FALSE(rtc_interface_->isActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_FALSE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));

  // Source lines 167-175: command activation after requested RUNNING status.
  commands.front().command.type = Command::ACTIVATE;
  update_cooperate_command_status(commands);
  EXPECT_TRUE(rtc_interface_->isRegistered(id));
  EXPECT_TRUE(rtc_interface_->isActivated(id));
  EXPECT_TRUE(rtc_interface_->isForceActivated(id));
  EXPECT_FALSE(rtc_interface_->isForceDeactivated(id));
  EXPECT_TRUE(rtc_interface_->isRTCEnabled(id));
  EXPECT_FALSE(rtc_interface_->isTerminated(id));
}
}  // namespace autoware::rtc_interface

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
