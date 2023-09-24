#ifndef JOY_TELEOP_HPP
#define JOY_TELEOP_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"
#include "quadruped_teleop/EMAFilter.hpp"

#include <memory>

class JoyTeleop : public rclcpp::Node
{
  public:
    JoyTeleop();
    ~JoyTeleop();

  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publishIKCallback();

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

    rclcpp::Publisher<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_publisher;

    rclcpp::TimerBase::SharedPtr _publish_ik_timer;

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
    double _filter_beta;

    std::map<std::string, uint8_t> _default_axis_linear_map;
    std::map<std::string, uint8_t> _default_axis_angular_map;
    std::map<std::string, uint8_t> _default_button_angular_map;
    std::map<std::string, double> _default_ik_limits;

};

#endif  // JOY_TELEOP_HPP
