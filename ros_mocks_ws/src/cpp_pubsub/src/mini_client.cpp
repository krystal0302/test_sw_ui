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

#include "rclcpp/rclcpp.hpp"
#include "ui_comm/srv/req_res.hpp"        

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 1) { 
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "argument number is NOT match!");      
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("req_res_client"); 
  rclcpp::Client<ui_comm::srv::ReqRes>::SharedPtr client =                        
    node->create_client<ui_comm::srv::ReqRes>("req_res_test");                  

  auto request = std::make_shared<ui_comm::srv::ReqRes::Request>();               
  request->req_msg = "task request";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
    RCLCPP_INFO(rclcpp::get_logger("ResReq"), "Result: %s", result.get()->res_msg.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ResReq"), "Failed to call req_res_test");    
  }

  rclcpp::shutdown();
  return 0;
}