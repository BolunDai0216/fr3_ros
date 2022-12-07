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

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <fr3_ros/controller_utils.h>

#include <Eigen/Dense>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace fr3_ros
{
class KinematicCBFController
  : public controller_interface::MultiInterfaceController<
        hardware_interface::VelocityJointInterface, franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  // define QP parameters
  proxsuite::proxqp::isize dim;
  proxsuite::proxqp::isize n_eq;
  proxsuite::proxqp::isize n_in;
  proxsuite::proxqp::dense::QP<double> qp;

  // define constructor and member initialization list
  KinematicCBFController() : dim(7), n_eq(0), n_in(1), qp(dim, n_eq, n_in){};

private:
  // pinocchio model & data
  pinocchio::Model model;
  pinocchio::Data data;

  // end-effector frame id in Pinocchio
  int ee_frame_id;

  // urdf file path for fr3 model
  std::string urdf_filename;

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  // publisher object for the log messages
  ros::Publisher control_log_publisher;

  // for logging data
  LogDataType logData;

  // initial position and orientation
  Eigen::Matrix<double, 3, 1> p_start;
  Eigen::Matrix<double, 3, 3> R_start;

  // terminal target position and orientation
  Eigen::Matrix<double, 3, 1> p_end;
  Eigen::Matrix<double, 3, 3> R_end;

  // end-effector targets during the trajectory
  Eigen::Matrix<double, 3, 1> p_target;
  Eigen::Matrix<double, 3, 1> v_target;
  Eigen::Matrix<double, 3, 3> R_target;
  Eigen::Matrix<double, 3, 1> w_target;

  // positional measurements
  Eigen::Matrix<double, 3, 1> p_current;
  Eigen::Matrix<double, 3, 3> R_current;

  // positional end-effector error
  Eigen::Matrix<double, 6, 1> P_error;

  // positional target
  Eigen::Matrix<double, 6, 1> dP_target;

  // commanded joint velocity
  Eigen::Matrix<double, 7, 1> dq_cmd;

  // Jacobian matrix and its pseudo-inverse
  Eigen::Matrix<double, 6, 7> jacobian;
  Eigen::MatrixXd pinv_jacobian;

  // trajectory configurations
  double traj_duration;
};

}  // namespace fr3_ros