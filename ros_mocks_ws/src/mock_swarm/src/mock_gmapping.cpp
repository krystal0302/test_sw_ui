#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <stdio.h>

using namespace std::chrono_literals;

class ParametersClass: public rclcpp::Node
{
  public:
    ParametersClass()
      : Node("far_gmapping")
    {
      this->declare_parameter<bool>("enable", false);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&ParametersClass::respond, this));
    }
    void respond()
    {
      this->get_parameter("enable", bEnable_);
      RCLCPP_INFO(this->get_logger(), "gmapping switch state: %d", bEnable_);
    }
  private:
    bool bEnable_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();
  return 0;
}