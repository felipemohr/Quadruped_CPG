#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "quadruped_kinematics/msg/leg_joints.hpp"
#include "quadruped_kinematics/srv/leg_ik.hpp"
#include "quadruped_kinematics/srv/quadruped_ik.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <cmath>

std::map<std::string, double> body_dimensions;
std::map<std::string, double> leg_dimensions;

Eigen::Matrix4d getTranslationMatrix(const float x, const float y, const float z)
{
  Eigen::Matrix4d translation_matrix;
  translation_matrix << 1.0, 0.0, 0.0, x,
                        0.0, 1.0, 0.0, y,
                        0.0, 0.0, 1.0, z,
                        0.0, 0.0, 0.0, 1.0;
  return translation_matrix;
}

Eigen::Matrix4d getTransformationMatrix(const geometry_msgs::msg::Vector3 translation,
                                        const geometry_msgs::msg::Vector3 rotation)
{
  Eigen::Vector3d translation_eigen;
  tf2::fromMsg(translation, translation_eigen);

  Eigen::AngleAxisd rollRotation(rotation.x, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchRotation(rotation.y, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawRotation(rotation.z, Eigen::Vector3d::UnitZ());

  Eigen::Quaternion q = rollRotation * pitchRotation * yawRotation;
  Eigen::Matrix3d rotation_eigen = q.toRotationMatrix();

  Eigen::Matrix4d Tm = Eigen::MatrixXd::Identity(4, 4);
  Tm.block<3, 3>(0, 0) = rotation_eigen;
  Tm.block<3, 1>(0, 3) = translation_eigen;

  return Tm;
}

Eigen::Matrix4d getBodyLegIK(const uint8_t leg,
                             const geometry_msgs::msg::Vector3 body_position,
                             const geometry_msgs::msg::Vector3 body_rotation)
{
  Eigen::Matrix4d Tm = getTransformationMatrix(body_position, body_rotation);

  Eigen::Matrix4d BodyLegMatrix;
  switch (leg)
  {
    case quadruped_kinematics::srv::LegIK_Request::FRONT_RIGHT_LEG:
      BodyLegMatrix = Tm * getTranslationMatrix( body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0);
      break;
    case quadruped_kinematics::srv::LegIK_Request::FRONT_LEFT_LEG:
      BodyLegMatrix = Tm * getTranslationMatrix( body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0);
      break;
    case quadruped_kinematics::srv::LegIK_Request::REAR_LEFT_LEG:
      BodyLegMatrix = Tm * getTranslationMatrix(-body_dimensions["L"]/2,  body_dimensions["W"]/2, 0.0);
      break;
    case quadruped_kinematics::srv::LegIK_Request::REAR_RIGHT_LEG:
      BodyLegMatrix = Tm * getTranslationMatrix(-body_dimensions["L"]/2, -body_dimensions["W"]/2, 0.0);
      break;
  }

  return BodyLegMatrix;
}

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
  leg_joints.hip_joint   =  theta1;
  leg_joints.thigh_joint = -theta2;
  leg_joints.calf_joint  = -theta3;

  return leg_joints;
}

quadruped_kinematics::msg::LegJoints computeLegIK(quadruped_kinematics::srv::LegIK::Request request)
{
  Eigen::Vector3d foot_point;
  tf2::fromMsg(request.foot_point, foot_point);

  // TODO: fix left if not use_foot_transform
  bool left;

  if (request.use_foot_transform)
  {
    switch (request.leg)
    {
      case quadruped_kinematics::srv::LegIK_Request::FRONT_RIGHT_LEG:
        foot_point = (getTranslationMatrix( body_dimensions["L"]/2, -(body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*foot_point.homogeneous()).head<3>();
        left = false;
        break;
      case quadruped_kinematics::srv::LegIK_Request::FRONT_LEFT_LEG:
        foot_point  = (getTranslationMatrix( body_dimensions["L"]/2,  (body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*foot_point.homogeneous()).head<3>();
        left = true;
        break;
      case quadruped_kinematics::srv::LegIK_Request::REAR_LEFT_LEG:
        foot_point   = (getTranslationMatrix(-body_dimensions["L"]/2,  (body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*foot_point.homogeneous()).head<3>();
        left = true;
        break;
      case quadruped_kinematics::srv::LegIK_Request::REAR_RIGHT_LEG:
        foot_point  = (getTranslationMatrix(-body_dimensions["L"]/2, -(body_dimensions["W"]/2+leg_dimensions["L1"]), -body_dimensions["H"])*foot_point.homogeneous()).head<3>();
        left = false;
        break;
    }
  }

  Eigen::Matrix4d foot_body_ik = getBodyLegIK(request.leg, request.body_translation, request.body_rotation);

  Eigen::Vector4d foot_point_ik = (foot_body_ik.inverse() * foot_point.homogeneous());

  geometry_msgs::msg::Point foot_point_msg = tf2::toMsg(Eigen::Vector3d(foot_point_ik.head(3)));

  quadruped_kinematics::msg::LegJoints leg_joints = getLegJoints(foot_point_msg, left);

  return leg_joints;
}

void computeLegIKCallback(const std::shared_ptr<quadruped_kinematics::srv::LegIK::Request> request,
                                std::shared_ptr<quadruped_kinematics::srv::LegIK::Response> response)
{
  response->leg_joints = computeLegIK(*request);
}

void computeQuadrupedIKCallback(const std::shared_ptr<quadruped_kinematics::srv::QuadrupedIK::Request> request,
                                      std::shared_ptr<quadruped_kinematics::srv::QuadrupedIK::Response> response)
{
  quadruped_kinematics::srv::LegIK::Request fr_request;
  fr_request.use_foot_transform = request->use_feet_transforms;
  fr_request.body_translation = request->body_translation;
  fr_request.body_rotation = request->body_rotation;
  quadruped_kinematics::srv::LegIK::Request fl_request = fr_request;
  quadruped_kinematics::srv::LegIK::Request rl_request = fr_request;
  quadruped_kinematics::srv::LegIK::Request rr_request = fr_request;

  fr_request.leg = quadruped_kinematics::srv::LegIK::Request::FRONT_RIGHT_LEG;
  fr_request.foot_point = request->front_right_foot;
  fl_request.leg = quadruped_kinematics::srv::LegIK::Request::FRONT_LEFT_LEG;
  fl_request.foot_point = request->front_left_foot;
  rl_request.leg = quadruped_kinematics::srv::LegIK::Request::REAR_LEFT_LEG;
  rl_request.foot_point = request->rear_left_foot;
  rr_request.leg = quadruped_kinematics::srv::LegIK::Request::REAR_RIGHT_LEG;
  rr_request.foot_point = request->rear_right_foot;

  response->front_right_joints = computeLegIK(fr_request);
  response->front_left_joints  = computeLegIK(fl_request);
  response->rear_left_joints   = computeLegIK(rl_request);
  response->rear_right_joints  = computeLegIK(rr_request);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ik_server");

  getQuadrupedParameters(node);

  rclcpp::Service<quadruped_kinematics::srv::LegIK>::SharedPtr leg_ik_service =
    node->create_service<quadruped_kinematics::srv::LegIK>("compute_leg_ik", &computeLegIKCallback);
  rclcpp::Service<quadruped_kinematics::srv::QuadrupedIK>::SharedPtr quadruped_ik_service =
    node->create_service<quadruped_kinematics::srv::QuadrupedIK>("compute_quadruped_ik", &computeQuadrupedIKCallback);
  
  RCLCPP_INFO(rclcpp::get_logger("ik_server"), "IK Server started.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
