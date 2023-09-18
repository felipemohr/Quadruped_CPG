#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"

#include <memory>

class GaitPlanner : public rclcpp::Node
{
  public:
    GaitPlanner();
    ~GaitPlanner();

  private:
    void publishIKCallback();

    rclcpp::Publisher<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_publisher;

    rclcpp::TimerBase::SharedPtr _publish_ik_timer;

    rclcpp::Time _start_time;

    float _d_step = 0.10;
    float _ground_clearance = 0.05;
    float _ground_penetration = 0.005;
    float _frequency = 5.0;

};
