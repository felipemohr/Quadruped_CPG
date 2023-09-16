#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/InverseKinematics.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

struct InverseKinematics::AllLegJoints
{
  quadruped_kinematics::msg::LegJoints front_right;
  quadruped_kinematics::msg::LegJoints front_left;
  quadruped_kinematics::msg::LegJoints back_left;
  quadruped_kinematics::msg::LegJoints back_right;
};

InverseKinematics::InverseKinematics() : Node("ik_node")
{
  RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Node initialized");

  _callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  _sub_options.callback_group = _callback_group;

  _cmd_ik_subscriber = this->create_subscription<quadruped_kinematics::msg::QuadrupedIK>("cmd_ik", 10, 
                              std::bind(&InverseKinematics::IKCallback, this, _1), _sub_options);
  _cmd_leg_ik_subscriber = this->create_subscription<quadruped_kinematics::msg::LegIK>("cmd_leg_ik", 10, 
                                  std::bind(&InverseKinematics::legIKCallback, this, _1), _sub_options);
  _cmd_default_pose_subscriber = this->create_subscription<std_msgs::msg::Empty>("cmd_default_pose", 10, 
                                        std::bind(&InverseKinematics::defaultPoseCallback, this, _1), _sub_options);

  _joint_commands_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

  _ik_client = this->create_client<quadruped_kinematics::srv::QuadrupedIK>("compute_quadruped_ik",
                                                                           rmw_qos_profile_services_default,
                                                                           _callback_group);
  _leg_ik_client = this->create_client<quadruped_kinematics::srv::LegIK>("compute_leg_ik",
                                                                         rmw_qos_profile_services_default,
                                                                         _callback_group);

  while (!_ik_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the compute_quadruped_ik service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "compute_quadruped_ik service not available, waiting again...");
  }

  while (!_leg_ik_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the compute_leg_ik service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "compute_leg_ik service not available, waiting again...");
  }

}

InverseKinematics::~InverseKinematics()
{
}

void InverseKinematics::IKCallback(const quadruped_kinematics::msg::QuadrupedIK::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_commands_msg;

  AllLegJoints all_leg_joints = this->computeIK(msg);

  // TODO: Include header

  joint_commands_msg.name.resize(12);
  joint_commands_msg.position.resize(12);

  joint_commands_msg.name.at(0) = "FR_hip_joint";
  joint_commands_msg.name.at(1) = "FR_thigh_joint";
  joint_commands_msg.name.at(2) = "FR_calf_joint";
  joint_commands_msg.position.at(0) = all_leg_joints.front_right.hip_joint;
  joint_commands_msg.position.at(1) = all_leg_joints.front_right.thigh_joint;
  joint_commands_msg.position.at(2) = all_leg_joints.front_right.calf_joint;

  joint_commands_msg.name.at(3) = "FL_hip_joint";
  joint_commands_msg.name.at(4) = "FL_thigh_joint";
  joint_commands_msg.name.at(5) = "FL_calf_joint";
  joint_commands_msg.position.at(3) = all_leg_joints.front_left.hip_joint;
  joint_commands_msg.position.at(4) = all_leg_joints.front_left.thigh_joint;
  joint_commands_msg.position.at(5) = all_leg_joints.front_left.calf_joint;

  joint_commands_msg.name.at(6) = "RL_hip_joint";
  joint_commands_msg.name.at(7) = "RL_thigh_joint";
  joint_commands_msg.name.at(8) = "RL_calf_joint";
  joint_commands_msg.position.at(6) = all_leg_joints.back_left.hip_joint;
  joint_commands_msg.position.at(7) = all_leg_joints.back_left.thigh_joint;
  joint_commands_msg.position.at(8) = all_leg_joints.back_left.calf_joint;

  joint_commands_msg.name.at(9)  = "RR_hip_joint";
  joint_commands_msg.name.at(10) = "RR_thigh_joint";
  joint_commands_msg.name.at(11) = "RR_calf_joint";
  joint_commands_msg.position.at(9)  = all_leg_joints.back_right.hip_joint;
  joint_commands_msg.position.at(10) = all_leg_joints.back_right.thigh_joint;
  joint_commands_msg.position.at(11) = all_leg_joints.back_right.calf_joint;

  _joint_commands_publisher->publish(joint_commands_msg);

}

void InverseKinematics::legIKCallback(const quadruped_kinematics::msg::LegIK::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_commands_msg;

  quadruped_kinematics::msg::LegJoints leg_joints = this->computeLegIK(msg);

  // TODO: Include header

  std::string leg_prefix;
  switch (msg->leg)
  {
  case quadruped_kinematics::msg::LegIK::FRONT_RIGHT_LEG:
    leg_prefix = "FR_";
    break;
  case quadruped_kinematics::msg::LegIK::FRONT_LEFT_LEG:
    leg_prefix = "FL_";
    break;
  case quadruped_kinematics::msg::LegIK::BACK_LEFT_LEG:
    leg_prefix = "RL_";
    break;
  case quadruped_kinematics::msg::LegIK::BACK_RIGHT_LEG:
    leg_prefix = "RR_";
    break;
  }  

  joint_commands_msg.name.resize(3);
  joint_commands_msg.position.resize(3);

  joint_commands_msg.name.at(0) = leg_prefix + "hip_joint";
  joint_commands_msg.name.at(1) = leg_prefix + "thigh_joint";
  joint_commands_msg.name.at(2) = leg_prefix + "calf_joint";
  joint_commands_msg.position.at(0) = leg_joints.hip_joint;
  joint_commands_msg.position.at(1) = leg_joints.thigh_joint;
  joint_commands_msg.position.at(2) = leg_joints.calf_joint;

  _joint_commands_publisher->publish(joint_commands_msg);

}

void InverseKinematics::defaultPoseCallback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void) msg;

  sensor_msgs::msg::JointState joint_commands_msg;

  joint_commands_msg.name.resize(12);
  joint_commands_msg.position.resize(12);

  joint_commands_msg.name.at(0) = "FR_hip_joint";
  joint_commands_msg.name.at(1) = "FR_thigh_joint";
  joint_commands_msg.name.at(2) = "FR_calf_joint";
  joint_commands_msg.name.at(3) = "FL_hip_joint";
  joint_commands_msg.name.at(4) = "FL_thigh_joint";
  joint_commands_msg.name.at(5) = "FL_calf_joint";
  joint_commands_msg.name.at(6) = "RL_hip_joint";
  joint_commands_msg.name.at(7) = "RL_thigh_joint";
  joint_commands_msg.name.at(8) = "RL_calf_joint";
  joint_commands_msg.name.at(9)  = "RR_hip_joint";
  joint_commands_msg.name.at(10) = "RR_thigh_joint";
  joint_commands_msg.name.at(11) = "RR_calf_joint";

  for (int i=0; i<(int)joint_commands_msg.position.size(); i++)
  {
    joint_commands_msg.position.at(i) = 0.0;
  }

  _joint_commands_publisher->publish(joint_commands_msg);

}

InverseKinematics::AllLegJoints InverseKinematics::computeIK(const quadruped_kinematics::msg::QuadrupedIK::SharedPtr msg)
{
  auto request = std::make_shared<quadruped_kinematics::srv::QuadrupedIK::Request>();
  
  request->use_feet_transforms = msg->use_feet_transforms;
  request->body_translation = msg->body_translation;
  request->body_rotation = msg->body_rotation;

  request->front_right_foot = msg->front_right_foot;
  request->front_left_foot  = msg->front_left_foot;
  request->back_left_foot   = msg->back_left_foot;
  request->back_right_foot  = msg->back_right_foot;

  auto response = this->_ik_client->async_send_request(request);
  std::future_status status = response.wait_for(10ms);

  AllLegJoints all_leg_joints;

  if (status == std::future_status::ready)
  {
    auto leg_joints = response.get();
    all_leg_joints.front_right = leg_joints->front_right_joints;
    all_leg_joints.front_left  = leg_joints->front_left_joints;
    all_leg_joints.back_left   = leg_joints->back_left_joints;
    all_leg_joints.back_right  = leg_joints->back_right_joints;
  }
  else
  {
    // TODO: Fix return and change warn to throttled
    RCLCPP_WARN(this->get_logger(), "Failed to compute leg joints, timeout");
  }

  return all_leg_joints;
}

quadruped_kinematics::msg::LegJoints InverseKinematics::computeLegIK(const quadruped_kinematics::msg::LegIK::SharedPtr msg)
{
  auto request = std::make_shared<quadruped_kinematics::srv::LegIK::Request>();
  
  request->use_foot_transform = msg->use_foot_transform;
  request->body_translation = msg->body_translation;
  request->body_rotation = msg->body_rotation;

  request->leg = msg->leg;
  request->foot_point = msg->foot_point;

  auto response = this->_leg_ik_client->async_send_request(request);
  std::future_status status = response.wait_for(10ms);

  quadruped_kinematics::msg::LegJoints leg_joints;

  if (status == std::future_status::ready)
  {
    leg_joints = response.get()->leg_joints;
  }
  else
  {
    // TODO: Fix return and change warn to throttled
    RCLCPP_WARN(this->get_logger(), "Failed to compute leg joints, timeout");
  }

  return leg_joints;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<InverseKinematics>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
