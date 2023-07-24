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
#include <nlohmann/json.hpp>

#include <memory>

using json = nlohmann::json;

void request_callback(const std::shared_ptr<ui_comm::srv::ReqRes::Request> request,     
          std::shared_ptr<ui_comm::srv::ReqRes::Response>       response)  
{
  // response->res_msg = "{\"status_code\": 1, \"message\": \"task removed successfully\"}";
  response->res_msg = request->req_msg;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "recieved request: %s", request->req_msg.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending request: %s", response->res_msg.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("req_res_server");  

  rclcpp::Service<ui_comm::srv::ReqRes>::SharedPtr service =                  
    node->create_service<ui_comm::srv::ReqRes>("req_res_test",  &request_callback);     

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to serve.");      

  rclcpp::spin(node);
  rclcpp::shutdown();
}