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
  _default_axis_angular_map = { {"yaw", 3}, {"pitch_positive", 4}, {"pitch_negative", 5} };
  _default_button_angular_map = { {"roll_positive", 4}, {"roll_negative", 5} };
  _default_ik_limits = { {"x", 0.1}, {"y", 0.1}, {"z", 0.1}, 
                         {"roll", 1.0}, {"pitch", 0.5}, {"yaw", 1.0} };

  _ik_msg.use_feet_transforms = true;
  _ik_msg_filtered.use_feet_transforms = true;

  this->declare_parameters("axis_linear", _default_axis_linear_map);
  this->declare_parameters("axis_angular", _default_axis_angular_map);
  this->declare_parameters("button_angular", _default_button_angular_map);
  this->declare_parameters("ik_limits", _default_ik_limits);
  this->declare_parameter("filter_beta", 0.9);

  this->get_parameters("axis_linear", _axis_linear_map);
  this->get_parameters("axis_angular", _axis_angular_map);
  this->get_parameters("button_angular", _button_angular_map);
  this->get_parameters("ik_limits", _ik_limits);
  this->get_parameter("filter_beta", _filter_beta);

  _body_translation_x.setFilterBeta(_filter_beta);
  _body_translation_y.setFilterBeta(_filter_beta);
  _body_translation_z.setFilterBeta(_filter_beta);
  _body_rotation_x.setFilterBeta(_filter_beta);
  _body_rotation_y.setFilterBeta(_filter_beta);
  _body_rotation_z.setFilterBeta(_filter_beta);

}

JoyTeleop::~JoyTeleop()
{
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  _ik_msg.body_translation.x = _ik_limits["x"] * msg->axes[_axis_linear_map["x"]];
  _ik_msg.body_translation.y = _ik_limits["y"] * msg->axes[_axis_linear_map["y"]];
  _ik_msg.body_translation.z = _ik_limits["z"] * msg->axes[_axis_linear_map["z"]];

  _ik_msg.body_rotation.x = _ik_limits["roll"] * (msg->buttons[_button_angular_map["roll_positive"]] 
                                                - msg->buttons[_button_angular_map["roll_negative"]]);
  _ik_msg.body_rotation.y = _ik_limits["pitch"] * ((-msg->axes[_axis_angular_map["pitch_positive"]] + 1)/2 
                                                 - (-msg->axes[_axis_angular_map["pitch_negative"]] + 1)/2);
  _ik_msg.body_rotation.z = _ik_limits["yaw"] * msg->axes[_axis_angular_map["yaw"]];
}

void JoyTeleop::publishIKCallback()
{
  _ik_msg_filtered.body_translation.x = _body_translation_x.filterData(_ik_msg.body_translation.x);
  _ik_msg_filtered.body_translation.y = _body_translation_y.filterData(_ik_msg.body_translation.y);
  _ik_msg_filtered.body_translation.z = _body_translation_z.filterData(_ik_msg.body_translation.z);

  _ik_msg_filtered.body_rotation.x = _body_rotation_x.filterData(_ik_msg.body_rotation.x);
  _ik_msg_filtered.body_rotation.y = _body_rotation_y.filterData(_ik_msg.body_rotation.y);
  _ik_msg_filtered.body_rotation.z = _body_rotation_z.filterData(_ik_msg.body_rotation.z);

  _cmd_ik_publisher->publish(_ik_msg_filtered);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
