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
  RCLCPP_INFO(this->get_logger(), "Inver Kinematics Node initialized");

  _cmd_ik_subscriber = this->create_subscription<quadruped_kinematics::msg::QuadrupedIK>("cmd_ik", 10, 
                              std::bind(&InverseKinematics::IKCallback, this, _1));
  _cmd_leg_ik_subscriber = this->create_subscription<quadruped_kinematics::msg::LegIK>("cmd_leg_ik", 10, 
                                  std::bind(&InverseKinematics::legIKCallback, this, _1));
  _cmd_default_pose_subscriber = this->create_subscription<std_msgs::msg::Empty>("cmd_default_pose", 10, 
                                        std::bind(&InverseKinematics::defaultPoseCallback, this, _1));

  _joint_commands_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);

  _ik_client = this->create_client<quadruped_kinematics::srv::QuadrupedIK>("compute_quadruped_ik");
  _leg_ik_client = this->create_client<quadruped_kinematics::srv::LegIK>("compute_leg_ik");

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

}

void InverseKinematics::legIKCallback(const quadruped_kinematics::msg::LegIK::SharedPtr msg)
{

}

void InverseKinematics::defaultPoseCallback(const std_msgs::msg::Empty::SharedPtr msg)
{

}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::shutdown();
  return 0;
}
