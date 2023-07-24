//  Copyright 2020 FARobot.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
#ifndef LOGGING_CORE__LOGGING_CORE_HPP_
#define LOGGING_CORE__LOGGING_CORE_HPP_

#include <sqlite3.h>
#include <stdio.h>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <sstream>

#include "far_fleet_msgs/msg/event.hpp"
#include "far_fleet_msgs/msg/event_array.hpp"
#include "far_fleet_msgs/msg/fleet_state.hpp"
#include "far_plan_msgs/msg/flow_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using Event = far_fleet_msgs::msg::Event;
using FleetState = far_fleet_msgs::msg::FleetState;
using FlowState = far_plan_msgs::msg::FlowState;

typedef boost::function<void(const std::string &)> LogMsgCallback;

class LoggingCore : public rclcpp::Node
{
public:
  LoggingCore();
  ~LoggingCore();

private:
  void systemLogCallback(Event::UniquePtr msg);
  void fleetStateCallback(FleetState::UniquePtr msg);
  void flowStateCallback(FlowState::UniquePtr msg);

  void flowProgressUpdateCallback();
  void flowFailStateCallback();
  void flowNewTasksCallback();

  void agentProgressUpdateCallback();
  void sysLoggerUpdateCallback();

  rclcpp::Subscription<far_fleet_msgs::msg::Event>::SharedPtr event_sub_;
  rclcpp::Subscription<far_fleet_msgs::msg::FleetState>::SharedPtr fleet_state_sub_;
  rclcpp::Subscription<far_plan_msgs::msg::FlowState>::SharedPtr flow_state_sub_;

  rclcpp::TimerBase::SharedPtr sys_logger_updater;
  rclcpp::TimerBase::SharedPtr flow_progress_updater;
  rclcpp::TimerBase::SharedPtr flow_fail_state_setter;
  rclcpp::TimerBase::SharedPtr flow_new_tasks_creator;

  rclcpp::TimerBase::SharedPtr agent_progress_updater;

  LogMsgCallback debug_, info_, error_, warning_;
  std::string format;
  boost::format fmt;
  std::string time;
  sqlite3 * syslogDB;
  sqlite3 * fleetStateDB;
  sqlite3 * flowStateDB;
  boost::uuids::random_generator generator;
};
#endif  // LOGGING_CORE__LOGGING_CORE_HPP_
