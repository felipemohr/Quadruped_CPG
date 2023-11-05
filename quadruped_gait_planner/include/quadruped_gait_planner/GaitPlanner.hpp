#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"
#include "quadruped_gait_planner/srv/gait_parameters.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <Eigen/Geometry>
#include <memory>

class GaitPlanner : public rclcpp::Node
{
  public:
    GaitPlanner();
    ~GaitPlanner();

  private:
    void setGaitParameters(const std::shared_ptr<quadruped_gait_planner::srv::GaitParameters::Request> request,
                                 std::shared_ptr<quadruped_gait_planner::srv::GaitParameters::Response> response);
    
    void publishIKCallback();
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    void updateGaitParameters();
    bool setGaitType(std::string gait_type);

    rclcpp::Service<quadruped_gait_planner::srv::GaitParameters>::SharedPtr _gait_parameters_service;

    rclcpp::Publisher<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_subscriber;

    rclcpp::TimerBase::SharedPtr _publish_ik_timer;

    rclcpp::Time _last_time;

    quadruped_kinematics::msg::QuadrupedIK _ik_msg;

    Eigen::Matrix4d _coupling_matrix;
    Eigen::Matrix4d _coupling_weights;

    Eigen::Vector4d _amplitude_mu;
    Eigen::Vector4d _frequency_omega;
    Eigen::Vector4d _convergence_factor_a;

    Eigen::Vector4d _amplitude_r;
    Eigen::Vector4d _amplitude_dr;
    Eigen::Vector4d _amplitude_d2r;

    Eigen::Vector4d _phase_theta;
    Eigen::Vector4d _phase_dtheta;

    Eigen::Vector4d _ground_multiplier;

    std::map<std::string, double> _gait_parameters;

    std::string _gait_type;

    Eigen::Vector4d _d_step_x;
    Eigen::Vector4d _d_step_y;
    float _ground_clearance;
    float _ground_penetration;

};
