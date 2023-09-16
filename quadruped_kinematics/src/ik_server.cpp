#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "quadruped_kinematics/msg/leg_joints.hpp"
#include "quadruped_kinematics/srv/leg_ik.hpp"
#include "quadruped_kinematics/srv/quadruped_ik.hpp"

#include <memory>
#include <cmath>

std::map<std::string, double> body_dimensions;
std::map<std::string, double> leg_dimensions;

void getQuadrupedParameters(rclcpp::Node::SharedPtr node)
{
  std::map<std::string, double> default_body_dimensions{ {"L", 0.2}, {"W", 0.15}, {"H", 0.15} };
  std::map<std::string, double> default_leg_dimensions{ {"L1", 0.05}, {"L2", 0.10}, {"L3", 0.10} };

  node->declare_parameters("body_dimensions", default_body_dimensions);
  node->declare_parameters("leg_dimensions", default_leg_dimensions);

  node->get_parameters("body_dimensions", body_dimensions);
  node->get_parameters("leg_dimensions", leg_dimensions);
}

quadruped_kinematics::msg::LegJoints getLegJoints(const geometry_msgs::msg::Point point, bool left=false)
{
  int reflect = left ? 1 : -1;

  float a = sqrt(pow(point.y, 2) + pow(point.z, 2) - pow(leg_dimensions["L1"], 2));
  float A = (pow(a, 2) + pow(point.x, 2) + pow(leg_dimensions["L2"], 2) - pow(leg_dimensions["L3"], 2)) / (2*leg_dimensions["L2"] * sqrt(pow(a, 2) + pow(point.x, 2)));
  float B = (pow(a, 2) + pow(point.x, 2) - pow(leg_dimensions["L2"], 2) - pow(leg_dimensions["L3"], 2)) / (2*leg_dimensions["L2"] * leg_dimensions["L3"]);

  float theta1 = atan2(point.y, -point.z) - atan2(reflect*leg_dimensions["L1"], a);
  float theta2 = M_PI_2 - atan2(a, point.x) - atan2(sqrt(1 - pow(A, 2)), A);
  float theta3 = atan2(sqrt(1 - pow(B, 2)), B);

  quadruped_kinematics::msg::LegJoints leg_joints;
  leg_joints.hip_joint   = theta1;
  leg_joints.thigh_joint = -(theta2 + M_PI_4);
  leg_joints.calf_joint  = -(theta3 - M_PI_2);

  return leg_joints;
}

void computeLegIK(const std::shared_ptr<quadruped_kinematics::srv::LegIK_Request> request,
                  std::shared_ptr<quadruped_kinematics::srv::LegIK_Response> responde)
{

}

void computeQuadrupedIK(const std::shared_ptr<quadruped_kinematics::srv::QuadrupedIK_Request> request,
                        std::shared_ptr<quadruped_kinematics::srv::QuadrupedIK_Response> response)
{
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Request> fr_request;
  fr_request->reference_link = request->reference_link;
  fr_request->body_translation = request->body_translation;
  fr_request->body_rotation = request->body_rotation;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Request> fl_request = fr_request;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Request> bl_request = fr_request;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Request> br_request = fr_request;

  fr_request->leg = quadruped_kinematics::srv::LegIK_Request::FRONT_RIGHT_LEG;
  fr_request->foot_point = request->front_right_foot;
  fl_request->leg = quadruped_kinematics::srv::LegIK_Request::FRONT_LEFT_LEG;
  fl_request->foot_point = request->front_left_foot;
  bl_request->leg = quadruped_kinematics::srv::LegIK_Request::BACK_LEFT_LEG;
  bl_request->foot_point = request->back_left_foot;
  br_request->leg = quadruped_kinematics::srv::LegIK_Request::BACK_RIGHT_LEG;
  br_request->foot_point = request->back_right_foot;

  std::shared_ptr<quadruped_kinematics::srv::LegIK_Response> fr_response;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Response> fl_response;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Response> bl_response;
  std::shared_ptr<quadruped_kinematics::srv::LegIK_Response> br_response;

  computeLegIK(fr_request, fr_response);
  computeLegIK(fl_request, fl_response);
  computeLegIK(bl_request, bl_response);
  computeLegIK(br_request, br_response);

  response->front_right_joints = fr_response->leg_joints;
  response->front_left_joints  = fl_response->leg_joints;
  response->back_left_joints   = bl_response->leg_joints;
  response->back_right_joints  = br_response->leg_joints;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ik_server");

  getQuadrupedParameters(node);

  rclcpp::Service<quadruped_kinematics::srv::LegIK>::SharedPtr leg_ik_service =
    node->create_service<quadruped_kinematics::srv::LegIK>("leg_ik", &computeLegIK);
  rclcpp::Service<quadruped_kinematics::srv::QuadrupedIK>::SharedPtr quadruped_ik_service =
    node->create_service<quadruped_kinematics::srv::QuadrupedIK>("quadruped_ik", &computeQuadrupedIK);
  
  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "IK Server started.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
