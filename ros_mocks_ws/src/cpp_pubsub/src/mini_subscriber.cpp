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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ui_comm/msg/ui_request.hpp"
#include "ui_comm/msg/ui_response.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<ui_comm::msg::UiRequest>(
      "/fb_0/ui_request", 10, std::bind(&MinimalSubscriber::uiRequest_callback, this, _1));
  }

private:
  void uiRequest_callback(const ui_comm::msg::UiRequest::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Recv Detection Request @(%.2f, %.2f) by method '%s'", msg->pose.x, msg->pose.y, msg->method.c_str());
  }
  rclcpp::Subscription<ui_comm::msg::UiRequest>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
