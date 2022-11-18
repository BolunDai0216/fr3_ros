#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <fr3_ros/pinocchio_utils.h>
#include <fr3_ros/controller_utils.h>

namespace fr3_ros {

class JointPDController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                                                hardware_interface::EffortJointInterface, 
                                                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // pinocchio model & data
  pinocchio::Model model;
  pinocchio::Data data;

  //publisher object for the log messages
  ros::Publisher control_log_publisher;
  LogDataType logData;

  // end-effector frame id in Pinocchio
  int ee_frame_id;

  // interface with franka_hw
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // clock only for controller
  double controlller_clock;

  // joint targets
  Eigen::Matrix<double, 7, 1> delta_q_target;

  // applied torque
  Eigen::Matrix<double, 7, 1> torques;

  // fixed rotation matrix target
  Eigen::Matrix<double, 3, 3> R_target;

  // changing position vector target
  Eigen::Matrix<double, 3, 1> p_target;
  Eigen::Matrix<double, 6, 1> dP_target;

  // measured end-effector configuration
  Eigen::Matrix<double, 3, 1> p_measured;
  Eigen::Matrix<double, 3, 3> R_measured;

  // end-effector Jacobian
  Eigen::Matrix<double, 6, 7> jacobian;

  // pseudo-inverse
  Eigen::MatrixXd pinv_jacobian;

  // define gains for PD controller
  Eigen::Matrix<double, 7, 7> Kp;
  Eigen::Matrix<double, 7, 7> Kd;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;

  double half_period = 3;
  double amplitude = 0.3;
};

}  // namespace fr3_ros