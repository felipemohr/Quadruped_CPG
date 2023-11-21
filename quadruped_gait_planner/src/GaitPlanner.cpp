#include "rclcpp/rclcpp.hpp"
#include "quadruped_gait_planner/GaitPlanner.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

GaitPlanner::GaitPlanner() : Node("gait_planner_node")
{
  RCLCPP_INFO(this->get_logger(), "Gait Planner node initialized");

  _enable_gait_planner_service = this->create_service<std_srvs::srv::Empty>("enable_gait_planner",
                                       std::bind(&GaitPlanner::enableGaitPlanner, this, _1, _2));
  _disable_gait_planner_service = this->create_service<std_srvs::srv::Empty>("disable_gait_planner",
                                        std::bind(&GaitPlanner::disableGaitPlanner, this, _1, _2));
  _gait_parameters_service = this->create_service<quadruped_gait_planner::srv::GaitParameters>("set_gait_parameters", 
                                   std::bind(&GaitPlanner::setGaitParameters, this, _1, _2));

  _cmd_ik_publisher = this->create_publisher<quadruped_kinematics::msg::QuadrupedIK>("cmd_ik", 10);
  _publish_ik_timer = this->create_wall_timer(10ms, std::bind(&GaitPlanner::publishIKCallback, this));

  _cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&GaitPlanner::cmdVelCallback, this, _1));

  _last_time = this->get_clock()->now();

  _amplitude_r = Eigen::Vector4d::Random();
  _phase_theta = Eigen::Vector4d::Random();

  _ik_msg.use_feet_transforms = true;  

  std::map<std::string, double> default_gait_parameters = { 
    {"coupling_weight", 1.0},
    {"amplitude", 1.0},
    {"swing_frequency", 2.5},
    {"stance_frequency", 1.5},
    {"convergence_factor", 50.0},
    {"ground_clearance", 0.05},
    {"ground_penetration", 0.005},
  };
  
  this->declare_parameter("gait_type", "trot");
  this->declare_parameters("gait_parameters", default_gait_parameters);

  this->get_parameter("gait_type", _gait_type);
  setGaitType(_gait_type);
  updateGaitParameters();

}

GaitPlanner::~GaitPlanner()
{
}

void GaitPlanner::publishIKCallback()
{
  float dt = this->get_clock()->now().seconds() - _last_time.seconds();

  _amplitude_d2r = _convergence_factor_a.array() * (_convergence_factor_a.array() / 4.0 * (_amplitude_mu.array() - _amplitude_r.array()) - _amplitude_dr.array());
  _amplitude_dr += _amplitude_d2r * dt;

  for (int i=0; i<4; i++)
  {
    _phase_theta(i) < M_PI ? _frequency_omega(i) = 2*M_PI * _gait_parameters["swing_frequency"] : _frequency_omega(i) = 2*M_PI * _gait_parameters["stance_frequency"];
    _phase_dtheta(i) = _frequency_omega(i);
    for (int j=0; j<4; j++)
      _phase_dtheta(i) += _amplitude_r(j) * _coupling_weights(i,j) * sin(_phase_theta(j) - _phase_theta(i) - _coupling_matrix(i,j));
  }

  _amplitude_r += _amplitude_dr * dt;
  _phase_theta += _phase_dtheta * dt;

  for (int i=0; i<4; i++)
  {
    _phase_theta(i) = std::fmod(_phase_theta(i), 2*M_PI);
    sin(_phase_theta(i)) > 0 ? _ground_multiplier(i) = _ground_clearance : _ground_multiplier(i) = _ground_penetration;
  }

  Eigen::Vector4d foot_x = -_d_step_x.array() * _amplitude_r.array() * _phase_theta.array().cos();
  Eigen::Vector4d foot_y = -_d_step_y.array() * _amplitude_r.array() * _phase_theta.array().cos();
  Eigen::Vector4d foot_z = _ground_multiplier.array() * _phase_theta.array().sin();

  _ik_msg.front_left_foot.x = foot_x(0);
  _ik_msg.front_left_foot.y = foot_y(0);
  _ik_msg.front_left_foot.z = foot_z(0);

  _ik_msg.front_right_foot.x = foot_x(1);
  _ik_msg.front_right_foot.y = foot_y(1);
  _ik_msg.front_right_foot.z = foot_z(1);

  _ik_msg.rear_left_foot.x = foot_x(2);
  _ik_msg.rear_left_foot.y = foot_y(2);
  _ik_msg.rear_left_foot.z = foot_z(2);

  _ik_msg.rear_right_foot.x = foot_x(3);
  _ik_msg.rear_right_foot.y = foot_y(3);
  _ik_msg.rear_right_foot.z = foot_z(3);
  
  _cmd_ik_publisher->publish(_ik_msg);

  _last_time = this->get_clock()->now();
}

void GaitPlanner::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  _d_step_x.setConstant(msg->linear.x * (1.0/_gait_parameters["stance_frequency"]) / 4.0);
  _d_step_y.setConstant(msg->linear.y * (1.0/_gait_parameters["stance_frequency"]) / 4.0);

  float delta_angular = 0.188 * msg->angular.z * (1.0/_gait_parameters["stance_frequency"]) / 4.0;
  
  _d_step_x(0) -= delta_angular;
  _d_step_y(0) += delta_angular;

  _d_step_x(1) += delta_angular;
  _d_step_y(1) += delta_angular;
  
  _d_step_x(2) -= delta_angular;
  _d_step_y(2) -= delta_angular;
  
  _d_step_x(3) += delta_angular;
  _d_step_y(3) -= delta_angular;

}

void GaitPlanner::enableGaitPlanner(const std_srvs::srv::Empty::Request::SharedPtr request,
                                          std_srvs::srv::Empty::Response::SharedPtr response)
{
  (void) request;
  (void) response;
  _publish_ik_timer->reset();
  RCLCPP_INFO(this->get_logger(), "Gait planner enabled");
}

void GaitPlanner::disableGaitPlanner(const std_srvs::srv::Empty::Request::SharedPtr request,
                                           std_srvs::srv::Empty::Response::SharedPtr response)
{
  (void) request;
  (void) response;
  _publish_ik_timer->cancel();
  RCLCPP_INFO(this->get_logger(), "Gait planner disabled");
}

void GaitPlanner::setGaitParameters(const std::shared_ptr<quadruped_gait_planner::srv::GaitParameters::Request> request,
                                          std::shared_ptr<quadruped_gait_planner::srv::GaitParameters::Response> response)
{
  this->set_parameters({rclcpp::Parameter("gait_type", request->gait_type),
                        rclcpp::Parameter("coupling_weight", request->coupling_weight),
                        rclcpp::Parameter("amplitude", request->amplitude),
                        rclcpp::Parameter("swing_frequency", request->swing_frequency),
                        rclcpp::Parameter("stance_frequency", request->stance_frequency),
                        rclcpp::Parameter("convergence_factor", request->convergence_factor),
                        rclcpp::Parameter("ground_clearance", request->ground_clearance),
                        rclcpp::Parameter("ground_penetration", request->ground_penetration)
                      });
  response->success = this->setGaitType(request->gait_type);
  if (response->success)
    this->updateGaitParameters();
}

bool GaitPlanner::setGaitType(std::string gait_type)
{
  if (gait_type == "trot")
  {
    _coupling_matrix << 0, M_PI, M_PI, 0,
                       -M_PI, 0, 0, -M_PI,
                       -M_PI, 0, 0, -M_PI,
                        0, M_PI, M_PI, 0;
  }
  else if (gait_type == "walk")
  {
    _coupling_matrix << 0, M_PI, M_PI_2, 3*M_PI_2,
                       -M_PI, 0, -M_PI_2, -3*M_PI_2,
                       -M_PI_2, M_PI_2, 0, -M_PI,
                       -3*M_PI_2, 3*M_PI_2, M_PI, 0;
  }
  else if (gait_type == "pace")
  {
    _coupling_matrix << 0, M_PI, M_PI, M_PI,
                       -M_PI, 0, -M_PI, 0,
                        0, M_PI, 0, M_PI,
                       -M_PI, 0, -M_PI, 0;
  }
  else if (gait_type == "gallop")
  {
    _coupling_matrix << 0, 0, -M_PI, -M_PI,
                        0, 0, -M_PI, -M_PI,
                        M_PI, M_PI, 0, 0,
                        M_PI, M_PI, 0, 0;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Impossible to set '%s' gait type. Ignoring...", gait_type.c_str());
    return false;
  }
  return true;
}

void GaitPlanner::updateGaitParameters()
{
  this->get_parameters("gait_parameters", _gait_parameters);

  _coupling_weights.setConstant(_gait_parameters["coupling_weight"]);
  _amplitude_mu.setConstant(_gait_parameters["amplitude"]);
  _frequency_omega.setConstant(2*M_PI * _gait_parameters["stance_frequency"]);
  _convergence_factor_a.setConstant(_gait_parameters["convergence_factor"]);
  _ground_clearance = _gait_parameters["ground_clearance"];
  _ground_penetration = _gait_parameters["ground_penetration"];
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
