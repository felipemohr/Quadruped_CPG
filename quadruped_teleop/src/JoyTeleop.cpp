#include "rclcpp/rclcpp.hpp"
#include "quadruped_teleop/JoyTeleop.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyTeleop::JoyTeleop() : Node("joy_teleop_node")
{
  RCLCPP_INFO(this->get_logger(), "Joy Teleop Node initialized");

  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10,
                            std::bind(&JoyTeleop::joyCallback, this, _1));

  _cmd_ik_publisher = this->create_publisher<quadruped_kinematics::msg::QuadrupedIK>("cmd_ik", 10);

  _publish_ik_timer = this->create_wall_timer(20ms, std::bind(&JoyTeleop::publishIKCallback, this));

  _default_axis_linear_map  = { {"x", 0}, {"y", 1}, {"z", 2} };
  _default_axis_angular_map = { {"yaw", 3} };

  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);

  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);

}

JoyTeleop::~JoyTeleop()
{
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "joyCallback");

}

void JoyTeleop::publishIKCallback()
{
  RCLCPP_INFO(this->get_logger(), "publishIKCallback");

}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
