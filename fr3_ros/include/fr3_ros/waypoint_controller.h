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

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace fr3_ros {

class WaypointController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                                                 hardware_interface::EffortJointInterface, 
                                                                                 franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

  // define QP parameters
  proxsuite::proxqp::isize dim;
  proxsuite::proxqp::isize n_eq;
  proxsuite::proxqp::isize n_in;
  proxsuite::proxqp::dense::QP<double> qp;

  // define constructor and member initialization list
  WaypointController() : dim(14), n_eq(7), n_in(0), qp(dim, n_eq, n_in) {};

 private:
  void computeSolverParameters(const Eigen::Matrix<double, 7, 1>& q, 
                               const Eigen::Matrix<double, 7, 1>& dq);
  
  void resetTarget(void);

  void computeEndEffectorTarget(const double& controlller_clock, const double& traj_duration);

  // pinocchio model & data
  pinocchio::Model model;
  pinocchio::Data data;

  // end-effector frame id in Pinocchio
  int ee_frame_id;

  // urdf file path for fr3 model
  std::string urdf_filename;

  // interface with franka_hw
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // clock only for controller
  double controlller_clock;

  // joint targets
  Eigen::Matrix<double, 7, 1> delta_q_target;
  Eigen::Matrix<double, 7, 1> ddq_cmd;

  // applied torque
  Eigen::Matrix<double, 7, 1> torques;

  // target position and orientation at each time step
  Eigen::Matrix<double, 3, 1> p_target;
  Eigen::Matrix<double, 3, 1> v_target;
  Eigen::Matrix<double, 3, 1> a_target;
  Eigen::Matrix<double, 3, 3> R_target;
  Eigen::Vector3d w_target;
  Eigen::Vector3d dw_target;

  // intial position and orientation
  Eigen::Matrix<double, 3, 1> p_start;
  Eigen::Matrix<double, 3, 3> R_start;

  // terminal target position and orientation
  Eigen::Matrix<double, 3, 1> p_end;
  Eigen::Matrix<double, 3, 3> R_end;

  // changing positional vector target
  Eigen::Matrix<double, 6, 1> dP_target;
  Eigen::Matrix<double, 6, 1> ddP_cmd;

  // errors at each time step
  Eigen::Matrix<double, 6, 1> P_error;
  Eigen::Vector3d rotvec_err;

  // measured end-effector configuration
  Eigen::Matrix<double, 3, 1> p_measured;
  Eigen::Matrix<double, 3, 3> R_measured;

  // end-effector Jacobian
  Eigen::Matrix<double, 6, 7> jacobian;
  Eigen::Matrix<double, 6, 7> djacobian;

  // pseudo-inverse
  Eigen::MatrixXd pinv_jacobian;

  // define gains for PD controller
  Eigen::Matrix<double, 7, 7> Kp;
  Eigen::Matrix<double, 7, 7> Kd;
  Eigen::Matrix<double, 6, 6> tKp;
  Eigen::Matrix<double, 6, 6> tKd;

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::vector<double> tk_gains_;
  std::vector<double> td_gains_;

  // QP parameters
  Eigen::Matrix<double, 14, 14> qp_H;
  Eigen::Matrix<double, 14, 1> qp_g;
  Eigen::Matrix<double, 7, 14> qp_A;
  Eigen::Matrix<double, 7, 1> qp_b;

  // QP problem parameters
  Eigen::Matrix<double, 7, 1> q_nominal;
  Eigen::Matrix<double, 7, 1> ddq_nominal;
  Eigen::Matrix<double, 6, 1> Jddq_desired;
  Eigen::Matrix<double, 7, 7> proj_mat;
  double epsilon;
  bool qp_initialized = false;

  // define trajectory
  std::array<Eigen::Matrix<double, 3, 1>, 3> waypoints;
  int waypoint_id;
  double traj_duration;
};

}  // namespace fr3_ros