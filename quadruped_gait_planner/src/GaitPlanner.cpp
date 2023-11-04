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

  _last_time = this->get_clock()->now();

  // Trot gait
  _coupling_matrix << 0, M_PI, M_PI, 0,
                     -M_PI, 0, 0, -M_PI,
                     -M_PI, 0, 0, -M_PI,
                      0, M_PI, M_PI, 0;

  // // Walk gait
  // _coupling_matrix << 0, M_PI, M_PI_2, 3*M_PI_2,
  //                     -M_PI, 0, -M_PI_2, -3*M_PI_2,
  //                     -M_PI_2, M_PI_2, 0, -M_PI,
  //                     -3*M_PI_2, 3*M_PI_2, M_PI, 0;

  // // Pace gait
  // _coupling_matrix << 0, M_PI, M_PI, M_PI,
  //                     -M_PI, 0, -M_PI, 0,
  //                     0, M_PI, 0, M_PI,
  //                     -M_PI, 0, -M_PI, 0;

  // // Gallop gait
  // _coupling_matrix << 0, 0, -M_PI, -M_PI,
  //                     0, 0, -M_PI, -M_PI,
  //                     M_PI, M_PI, 0, 0,
  //                     M_PI, M_PI, 0, 0;

  _amplitude_r = Eigen::Vector4d::Random();
  _phase_theta = Eigen::Vector4d::Random();

  _coupling_weights.setOnes();

  _amplitude_mu.setConstant(1.5);
  _frequency_omega.setConstant(1.0 * 2 * M_PI);
  _convergence_factor_a.setConstant(50.0);

  _ik_msg.use_feet_transforms = true;

}

GaitPlanner::~GaitPlanner()
{
  
}

void GaitPlanner::publishIKCallback()
{
  float dt = this->get_clock()->now().seconds() - _last_time.seconds();

  _amplitude_d2r = _convergence_factor_a.array() * (_convergence_factor_a.array() / 4.0 * (_amplitude_mu.array() - _amplitude_r.array()) - _amplitude_dr.array());
  _amplitude_dr += _amplitude_d2r * dt;

  _phase_dtheta = _frequency_omega;
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
        _phase_dtheta(i) += _amplitude_r(j) * _coupling_weights(i,j) * sin(_phase_theta(j) - _phase_theta(i) - _coupling_matrix(i,j));

  _amplitude_r += _amplitude_dr * dt;
  _phase_theta += _phase_dtheta * dt;

  for (int i=0; i<4; i++)
    sin(_phase_theta(i)) > 0 ? _ground_multiplier(i) = _ground_clearance : _ground_multiplier(i) = _ground_penetration;

  Eigen::Vector4d foot_x = -_d_step * (_amplitude_r.array() - Eigen::Vector4d::Ones().array()) * _phase_theta.array().cos();
  Eigen::Vector4d foot_z = _ground_multiplier.array() * _phase_theta.array().sin();

  _ik_msg.front_left_foot.x = foot_x(0);
  _ik_msg.front_left_foot.z = foot_z(0);

  _ik_msg.front_right_foot.x = foot_x(1);
  _ik_msg.front_right_foot.z = foot_z(1);

  _ik_msg.rear_left_foot.x = foot_x(2);
  _ik_msg.rear_left_foot.z = foot_z(2);

  _ik_msg.rear_right_foot.x = foot_x(3);
  _ik_msg.rear_right_foot.z = foot_z(3);
  
  _cmd_ik_publisher->publish(_ik_msg);

  _last_time = this->get_clock()->now();
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
