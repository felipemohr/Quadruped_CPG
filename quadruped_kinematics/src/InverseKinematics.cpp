#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/InverseKinematics.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

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

  _last_leg_joints.hip_joint   = 0.0;
  _last_leg_joints.thigh_joint = 0.7854;
  _last_leg_joints.calf_joint  = -1.5708;

  _last_quadruped_joints.front_right_joints = _last_leg_joints;
  _last_quadruped_joints.front_left_joints  = _last_leg_joints;
  _last_quadruped_joints.rear_left_joints   = _last_leg_joints;
  _last_quadruped_joints.rear_right_joints  = _last_leg_joints;

  std::map<std::string, double> default_range = { {"lower_limit", -0.7854}, {"upper_limit", 0.7854} };
  this->declare_parameters("hip_joint_range", default_range);
  this->declare_parameters("thigh_joint_range", default_range);
  this->declare_parameters("calf_joint_range", default_range);

  this->get_parameters("hip_joint_range", _hip_joint_range);
  this->get_parameters("thigh_joint_range", _thigh_joint_range);
  this->get_parameters("calf_joint_range", _calf_joint_range);

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

  quadruped_kinematics::msg::QuadrupedJoints quadruped_joints = this->computeIK(msg);

  if (!checkJointAnglesNaN(quadruped_joints.front_right_joints) ||
      !checkJointAnglesNaN(quadruped_joints.front_left_joints)  ||
      !checkJointAnglesNaN(quadruped_joints.rear_left_joints)   ||
      !checkJointAnglesNaN(quadruped_joints.rear_right_joints)
      )
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "NaN value computed, impossible to set IK");
    return;
  }
  if (!checkJointAnglesRange(quadruped_joints.front_right_joints) ||
      !checkJointAnglesRange(quadruped_joints.front_left_joints)  ||
      !checkJointAnglesRange(quadruped_joints.rear_left_joints)   ||
      !checkJointAnglesRange(quadruped_joints.rear_right_joints)
     )
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Leg joints angle out of range");
    return;
  }

  joint_commands_msg.header.frame_id = "";
  joint_commands_msg.header.stamp = this->get_clock()->now();

  joint_commands_msg.name.resize(12);
  joint_commands_msg.position.resize(12);

  joint_commands_msg.name.at(0) = "FR_hip_joint";
  joint_commands_msg.name.at(1) = "FR_thigh_joint";
  joint_commands_msg.name.at(2) = "FR_calf_joint";
  joint_commands_msg.position.at(0) = quadruped_joints.front_right_joints.hip_joint;
  joint_commands_msg.position.at(1) = quadruped_joints.front_right_joints.thigh_joint;
  joint_commands_msg.position.at(2) = quadruped_joints.front_right_joints.calf_joint;

  joint_commands_msg.name.at(3) = "FL_hip_joint";
  joint_commands_msg.name.at(4) = "FL_thigh_joint";
  joint_commands_msg.name.at(5) = "FL_calf_joint";
  joint_commands_msg.position.at(3) = quadruped_joints.front_left_joints.hip_joint;
  joint_commands_msg.position.at(4) = quadruped_joints.front_left_joints.thigh_joint;
  joint_commands_msg.position.at(5) = quadruped_joints.front_left_joints.calf_joint;

  joint_commands_msg.name.at(6) = "RL_hip_joint";
  joint_commands_msg.name.at(7) = "RL_thigh_joint";
  joint_commands_msg.name.at(8) = "RL_calf_joint";
  joint_commands_msg.position.at(6) = quadruped_joints.rear_left_joints.hip_joint;
  joint_commands_msg.position.at(7) = quadruped_joints.rear_left_joints.thigh_joint;
  joint_commands_msg.position.at(8) = quadruped_joints.rear_left_joints.calf_joint;

  joint_commands_msg.name.at(9)  = "RR_hip_joint";
  joint_commands_msg.name.at(10) = "RR_thigh_joint";
  joint_commands_msg.name.at(11) = "RR_calf_joint";
  joint_commands_msg.position.at(9)  = quadruped_joints.rear_right_joints.hip_joint;
  joint_commands_msg.position.at(10) = quadruped_joints.rear_right_joints.thigh_joint;
  joint_commands_msg.position.at(11) = quadruped_joints.rear_right_joints.calf_joint;

  _joint_commands_publisher->publish(joint_commands_msg);

}

void InverseKinematics::legIKCallback(const quadruped_kinematics::msg::LegIK::SharedPtr msg)
{
  sensor_msgs::msg::JointState joint_commands_msg;

  quadruped_kinematics::msg::LegJoints leg_joints = this->computeLegIK(msg);

  if (!checkJointAnglesNaN(leg_joints))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "NaN value computed, impossible to set IK");
    return;
  }
  if (!checkJointAnglesRange(leg_joints))
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Leg joints angle out of range");
    return;
  }

  std::string leg_prefix;
  switch (msg->leg)
  {
  case quadruped_kinematics::msg::LegIK::FRONT_RIGHT_LEG:
    leg_prefix = "FR_";
    break;
  case quadruped_kinematics::msg::LegIK::FRONT_LEFT_LEG:
    leg_prefix = "FL_";
    break;
  case quadruped_kinematics::msg::LegIK::REAR_LEFT_LEG:
    leg_prefix = "RL_";
    break;
  case quadruped_kinematics::msg::LegIK::REAR_RIGHT_LEG:
    leg_prefix = "RR_";
    break;
  }

  joint_commands_msg.header.frame_id = "";
  joint_commands_msg.header.stamp = this->get_clock()->now();

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

  joint_commands_msg.header.frame_id = "";
  joint_commands_msg.header.stamp = this->get_clock()->now();

  joint_commands_msg.name.resize(12);
  joint_commands_msg.position.resize(12);

  joint_commands_msg.name.at(0) = "FR_hip_joint";
  joint_commands_msg.name.at(1) = "FR_thigh_joint";
  joint_commands_msg.name.at(2) = "FR_calf_joint";
  joint_commands_msg.position.at(0) = 0.0;
  joint_commands_msg.position.at(1) = 0.7854;
  joint_commands_msg.position.at(2) = -1.5708;

  joint_commands_msg.name.at(3) = "FL_hip_joint";
  joint_commands_msg.name.at(4) = "FL_thigh_joint";
  joint_commands_msg.name.at(5) = "FL_calf_joint";
  joint_commands_msg.position.at(3) = 0.0;
  joint_commands_msg.position.at(4) = 0.7854;
  joint_commands_msg.position.at(5) = -1.5708;

  joint_commands_msg.name.at(6) = "RL_hip_joint";
  joint_commands_msg.name.at(7) = "RL_thigh_joint";
  joint_commands_msg.name.at(8) = "RL_calf_joint";
  joint_commands_msg.position.at(6) = 0.0;
  joint_commands_msg.position.at(7) = 0.7854;
  joint_commands_msg.position.at(8) = -1.5708;

  joint_commands_msg.name.at(9)  = "RR_hip_joint";
  joint_commands_msg.name.at(10) = "RR_thigh_joint";
  joint_commands_msg.name.at(11) = "RR_calf_joint";
  joint_commands_msg.position.at(9) = 0.0;
  joint_commands_msg.position.at(10) = 0.7854;
  joint_commands_msg.position.at(11) = -1.5708;

  _joint_commands_publisher->publish(joint_commands_msg);

}

quadruped_kinematics::msg::QuadrupedJoints InverseKinematics::computeIK(const quadruped_kinematics::msg::QuadrupedIK::SharedPtr msg)
{
  auto request = std::make_shared<quadruped_kinematics::srv::QuadrupedIK::Request>();
  
  request->use_feet_transforms = msg->use_feet_transforms;
  request->body_translation = msg->body_translation;
  request->body_rotation = msg->body_rotation;

  request->front_right_foot = msg->front_right_foot;
  request->front_left_foot  = msg->front_left_foot;
  request->rear_left_foot   = msg->rear_left_foot;
  request->rear_right_foot  = msg->rear_right_foot;

  // auto start = std::chrono::steady_clock::now();

  auto result = this->_ik_client->async_send_request(request);
  std::future_status status = result.wait_for(5ms);

  quadruped_kinematics::msg::QuadrupedJoints quadruped_joints;

  if (status == std::future_status::ready)
  {
    auto leg_joints = result.get();
    quadruped_joints.front_right_joints = leg_joints->front_right_joints;
    quadruped_joints.front_left_joints  = leg_joints->front_left_joints;
    quadruped_joints.rear_left_joints   = leg_joints->rear_left_joints;
    quadruped_joints.rear_right_joints  = leg_joints->rear_right_joints;
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Failed to compute quadruped joints, timeout");
    return _last_quadruped_joints;
  }

  // auto end = std::chrono::steady_clock::now();
  // std::cout << "Elapsed time in microseconds: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
  //           << " µs" << std::endl;

  _last_quadruped_joints = quadruped_joints;

  return quadruped_joints;
}

quadruped_kinematics::msg::LegJoints InverseKinematics::computeLegIK(const quadruped_kinematics::msg::LegIK::SharedPtr msg)
{
  auto request = std::make_shared<quadruped_kinematics::srv::LegIK::Request>();
  
  request->use_foot_transform = msg->use_foot_transform;
  request->body_translation = msg->body_translation;
  request->body_rotation = msg->body_rotation;

  request->leg = msg->leg;
  request->foot_point = msg->foot_point;

  // auto start = std::chrono::steady_clock::now();

  auto result = this->_leg_ik_client->async_send_request(request);
  std::future_status status = result.wait_for(5ms);

  quadruped_kinematics::msg::LegJoints leg_joints;

  if (status == std::future_status::ready)
  {
    leg_joints = result.get()->leg_joints;
  }
  else
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Failed to compute leg joints, timeout");
    return _last_leg_joints;
  }

  // auto end = std::chrono::steady_clock::now();
  // std::cout << "Elapsed time in microseconds: "
  //           << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
  //           << " µs" << std::endl;

  _last_leg_joints = leg_joints;

  return leg_joints;
}

bool InverseKinematics::checkJointAnglesNaN(const quadruped_kinematics::msg::LegJoints leg_joints)
{
  return !(std::isnan(leg_joints.hip_joint)   || 
           std::isnan(leg_joints.thigh_joint) || 
           std::isnan(leg_joints.calf_joint)
          );
}

bool InverseKinematics::checkJointAnglesRange(const quadruped_kinematics::msg::LegJoints leg_joints)
{
  return ( (leg_joints.hip_joint   >= _hip_joint_range["lower_limit"]   && leg_joints.hip_joint   <= _hip_joint_range["upper_limit"])   &&
           (leg_joints.thigh_joint >= _thigh_joint_range["lower_limit"] && leg_joints.thigh_joint <= _thigh_joint_range["upper_limit"]) &&
           (leg_joints.calf_joint  >= _calf_joint_range["lower_limit"]  && leg_joints.calf_joint  <= _calf_joint_range["upper_limit"]) 
         );
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
