#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/msg/leg_joints.hpp"
#include "quadruped_kinematics/srv/leg_ik.hpp"
#include "quadruped_kinematics/msg/leg_ik.hpp"
#include "quadruped_kinematics/srv/quadruped_ik.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/empty.hpp"

#include <memory>

class InverseKinematics : public rclcpp::Node
{
  public:
    InverseKinematics();
    ~InverseKinematics();

  private:
    void IKCallback(const quadruped_kinematics::msg::QuadrupedIK::SharedPtr msg);
    void legIKCallback(const quadruped_kinematics::msg::LegIK::SharedPtr msg);
    void defaultPoseCallback(const std_msgs::msg::Empty::SharedPtr msg);

    rclcpp::Subscription<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_subscriber;
    rclcpp::Subscription<quadruped_kinematics::msg::LegIK>::SharedPtr _cmd_leg_ik_subscriber;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr _cmd_default_pose_subscriber;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _joint_commands_publisher;

    rclcpp::Client<quadruped_kinematics::srv::QuadrupedIK>::SharedPtr _ik_client;
    rclcpp::Client<quadruped_kinematics::srv::LegIK>::SharedPtr _leg_ik_client;

    sensor_msgs::msg::JointState _joint_commands_msg;

};
