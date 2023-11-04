#include "rclcpp/rclcpp.hpp"
#include "quadruped_kinematics/msg/quadruped_ik.hpp"

#include <Eigen/Geometry>
#include <memory>

class GaitPlanner : public rclcpp::Node
{
  public:
    GaitPlanner();
    ~GaitPlanner();

  private:
    void publishIKCallback();
    void setGaitType(std::string gait_type);
    void updateGaitParameters();

    rclcpp::Publisher<quadruped_kinematics::msg::QuadrupedIK>::SharedPtr _cmd_ik_publisher;

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

    float _d_step;
    float _ground_clearance;
    float _ground_penetration;

};
