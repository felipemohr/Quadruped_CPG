#include "rclcpp/rclcpp.hpp"
#include "quadruped_gait_planner/GaitPlanner.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

GaitPlanner::GaitPlanner() : Node("gait_planner_node")
{
  RCLCPP_INFO(this->get_logger(), "Gait Planner node initialized");

  _cmd_ik_publisher = this->create_publisher<quadruped_kinematics::msg::QuadrupedIK>("cmd_ik", 10);
  _publish_ik_timer = this->create_wall_timer(10ms, std::bind(&GaitPlanner::publishIKCallback, this));

  _start_time = this->get_clock()->now();

}

GaitPlanner::~GaitPlanner()
{
  
}

void GaitPlanner::publishIKCallback()
{
  float elapsed_time = this->get_clock()->now().seconds() - _start_time.seconds();

  quadruped_kinematics::msg::QuadrupedIK ik_msg;
  ik_msg.use_feet_transforms = true;

  float sine_fr = sin(2*M_PI*_frequency * elapsed_time);
  float sine_fl = sin(2*M_PI*_frequency * elapsed_time + M_PI);
  float sine_rl = sin(2*M_PI*_frequency * elapsed_time + M_PI/2);
  float sine_rr = sin(2*M_PI*_frequency * elapsed_time + 3*M_PI/2);
  
  float g1 = sine_fr > 0 ? _ground_clearance : _ground_penetration;
  float g2 = sine_fl > 0 ? _ground_clearance : _ground_penetration;
  float g3 = sine_rl > 0 ? _ground_clearance : _ground_penetration;
  float g4 = sine_rr > 0 ? _ground_clearance : _ground_penetration;
  
  ik_msg.front_right_foot.z = g1 * sine_fr;
  ik_msg.front_right_foot.x = _d_step * sine_fr;

  ik_msg.front_left_foot.z = g2 * sine_fl;
  ik_msg.front_left_foot.x = _d_step * sine_fl;

  ik_msg.rear_left_foot.z = g3 * sine_rl;
  ik_msg.rear_left_foot.x = _d_step * sine_rl;

  ik_msg.rear_right_foot.z = g4 * sine_rr;
  ik_msg.rear_right_foot.x = _d_step * sine_rr;

  _cmd_ik_publisher->publish(ik_msg);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
