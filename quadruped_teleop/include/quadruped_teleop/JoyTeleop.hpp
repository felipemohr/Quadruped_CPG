#ifndef JOY_TELEOP_HPP
#define JOY_TELEOP_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"
#include "quadruped_teleop/EMAFilter.hpp"

#include <memory>

enum class TeleopState
{
  WALKING,
  MOVING_BODY
};

class JoyTeleop : public rclcpp::Node
{
  public:
    JoyTeleop();
    ~JoyTeleop();

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publishVelCallback();
    void publishIKCallback();

    void readVelMsg(const sensor_msgs::msg::Joy::SharedPtr msg);
    void readIKMsg(const sensor_msgs::msg::Joy::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_publisher;
    rclcpp::Publisher<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_publisher;

    rclcpp::TimerBase::SharedPtr _publish_vel_timer;
    rclcpp::TimerBase::SharedPtr _publish_ik_timer;

    geometry_msgs::msg::Twist _vel_msg;
    quadruped_kinematics::msg::QuadrupedIK _ik_msg;
    quadruped_kinematics::msg::QuadrupedIK _ik_msg_filtered;

    EMAFilter _body_translation_x;
    EMAFilter _body_translation_y;
    EMAFilter _body_translation_z;
    EMAFilter _body_rotation_x;
    EMAFilter _body_rotation_y;
    EMAFilter _body_rotation_z;

    std::map<std::string, uint8_t> _axis_linear_map;
    std::map<std::string, uint8_t> _axis_angular_map;
    std::map<std::string, uint8_t> _button_angular_map;
    std::map<std::string, double> _ik_limits;
    std::map<std::string, double> _vel_limits;
    uint8_t _change_state_map;
    double _filter_beta;

    std::map<std::string, uint8_t> _default_axis_linear_map;
    std::map<std::string, uint8_t> _default_axis_angular_map;
    std::map<std::string, uint8_t> _default_button_angular_map;
    std::map<std::string, double> _default_ik_limits;
    std::map<std::string, double> _default_vel_limits;

    TeleopState _last_state;
    TeleopState _state;

};

#endif  // JOY_TELEOP_HPP
