#include <fr3_ros/qp_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <fr3_ros/pinocchio_utils.h>
#include <fr3_ros/controller_utils.h>

#include <cmath>
#include <optional>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>

namespace pin = pinocchio;
namespace ps = proxsuite::proxqp;

namespace fr3_ros {

bool QPController::init(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("QPController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("QPController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("QPController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "QPController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "QPController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "QPController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "QPController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "QPController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "QPController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "QPController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "QPController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("tk_gains", tk_gains_) || tk_gains_.size() != 6) {
    ROS_ERROR(
        "QPController:  Invalid or no tk_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("td_gains", td_gains_) || td_gains_.size() != 6) {
    ROS_ERROR(
        "QPController:  Invalid or no td_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  if (!node_handle.getParam("urdf_filename", urdf_filename)) {
    ROS_ERROR_STREAM("QPController: Could not read parameter urdf_filename");
    return false;
  }

  if (!node_handle.getParam("epsilon", epsilon)) {
    ROS_ERROR_STREAM("QPController: Could not read parameter epsilon");
    return false;
  }

  // build pin_robot from urdf
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  return true;
}

void QPController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_init(initial_state.dq.data());

  updatePinocchioModel(model, data, q_init, dq_init);

  // define end-effector frame id in pinocchio
  ee_frame_id = model.getFrameId("fr3_hand_tcp");

  // get current end-effector position and orientation
  p_target = data.oMf[ee_frame_id].translation();
  R_target = data.oMf[ee_frame_id].rotation();

  dP_target << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ddP_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  q_nominal = q_init;

  // get Kp and Kd gains
  readJointPDGains(k_gains_, d_gains_, Kp, Kd);
  readTaskPDGains(tk_gains_, td_gains_, tKp, tKd);
  
  // initialize clock
  controlller_clock = 0.0;
}

void QPController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  static ps::dense::QP<double> qp(dim, n_eq, n_in);

  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  updatePinocchioModel(model, data, q, dq);

  // measure end-effector position and orientation
  p_measured = data.oMf[ee_frame_id].translation();
  R_measured = data.oMf[ee_frame_id].rotation();

  // get end-effector jacobian
  pin::getFrameJacobian(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, jacobian);

  // get time derivative of positional Jacobian
  pin::getFrameJacobianTimeVariation(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, djacobian);

  // compute orientation error with targets
  Eigen::Matrix<double, 3, 3> R_error = R_target * R_measured.transpose();
  Eigen::AngleAxisd AngleAxisErr(R_error);
  Eigen::Vector3d rotvec_err = AngleAxisErr.axis() * AngleAxisErr.angle();

  // compute new p_target along the y-axis
  p_target[1] = std::sin(M_PI * controlller_clock / half_period) * amplitude;

  // compute new dP_target along the y-axis
  dP_target[1] = (M_PI / half_period) * std::cos(M_PI * controlller_clock / half_period) * amplitude;

  // compute new ddP_cmd along the y-axis
  ddP_cmd[1] = (M_PI * M_PI / (half_period * half_period)) * std::cos(M_PI * controlller_clock / half_period) * amplitude;

  // compute positional error
  Eigen::Matrix<double, 6, 1> P_error;
  P_error << p_target - p_measured, rotvec_err;

  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  // compute joint torque
  Jddq_desired = ddP_cmd + tKp * P_error + tKd * (dP_target - jacobian * dq) - djacobian * dq;
  proj_mat = Eigen::MatrixXd::Identity(7, 7) - pinv_jacobian * jacobian;
  ddq_nominal = Kp * (q_nominal - q) - Kd * dq;

  qp_H = 2 * (jacobian.transpose() * jacobian + epsilon * proj_mat.transpose() * proj_mat);
  qp_g = -2 * (Jddq_desired.transpose() * jacobian + epsilon * ddq_nominal.transpose() * proj_mat.transpose() * proj_mat).transpose();

  if (qp_initialized) {
    qp.update(qp_H, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  } else {
    qp.init(qp_H, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
    qp_initialized = true;
  }
  qp.solve();

  auto sol_ddq = qp.results.x.topLeftCorner(7, 1);
  auto sol_torque = qp.results.x.bottomLeftCorner(7, 1);

  // get M, h, g
  getDynamicsParameter(model, data, q, dq);

  torques = data.M * sol_ddq + (data.nle - data.g);

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }
}

void QPController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::QPController, controller_interface::ControllerBase)