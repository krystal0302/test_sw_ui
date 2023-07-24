// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ui_comm/msg/task_state.hpp"
#include "ui_comm/msg/task_state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MockFlow : public rclcpp::Node
{
public:
  MockFlow()
  : Node("mock_plan")
  {
    taskState_pub_ = this->create_publisher<ui_comm::msg::TaskState>("/task_state", 10);
    removeTask_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/remove_task", 10, std::bind(&MockFlow::removeTask_callback, this, _1));

    timer_ = this->create_wall_timer(500ms, std::bind(&MockFlow::timer_callback, this));

  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "TaskState sent!");
  }
  void removeTask_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Conducting RemoveTask Request...%s", msg->data.c_str() );
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ui_comm::msg::TaskState>::SharedPtr taskState_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr removeTask_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockFlow>());
  rclcpp::shutdown();
  return 0;
}
