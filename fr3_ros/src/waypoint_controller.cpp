#include <fr3_ros/waypoint_controller.h>
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

bool WaypointController::init(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("WaypointController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("WaypointController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("WaypointController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "WaypointController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "WaypointController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "WaypointController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "WaypointController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "WaypointController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "WaypointController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "WaypointController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "WaypointController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("tk_gains", tk_gains_) || tk_gains_.size() != 6) {
    ROS_ERROR(
        "WaypointController:  Invalid or no tk_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("td_gains", td_gains_) || td_gains_.size() != 6) {
    ROS_ERROR(
        "WaypointController:  Invalid or no td_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  if (!node_handle.getParam("urdf_filename", urdf_filename)) {
    ROS_ERROR_STREAM("WaypointController: Could not read parameter urdf_filename");
    return false;
  }

  if (!node_handle.getParam("epsilon", epsilon)) {
    ROS_ERROR_STREAM("WaypointController: Could not read parameter epsilon");
    return false;
  }

  // build pin_robot from urdf
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  return true;
}

void WaypointController::starting(const ros::Time& /* time */) {
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

  // initialize qp parameters with zero
  qp_H = Eigen::MatrixXd::Zero(14, 14);
  qp_g = Eigen::MatrixXd::Zero(14, 1);
  qp_A = Eigen::MatrixXd::Zero(7, 14);
  qp_b = Eigen::MatrixXd::Zero(7, 1);

  q_nominal = q_init;

  // get Kp and Kd gains
  readJointPDGains(k_gains_, d_gains_, Kp, Kd);
  readTaskPDGains(tk_gains_, td_gains_, tKp, tKd);
  
  // initialize clock
  controlller_clock = 0.0;
}

void WaypointController::update(const ros::Time& /*time*/, const ros::Duration& period) {
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

  // compute orientation error
  rotvec_err = computeRotVecError(R_target, R_measured);

  // compute new p_target, dP_target, ddP_cmd along the y-axis
  p_target[1] = std::sin(M_PI * controlller_clock / half_period) * amplitude;
  dP_target[1] = (M_PI / half_period) * std::cos(M_PI * controlller_clock / half_period) * amplitude;
  ddP_cmd[1] = (M_PI * M_PI / (half_period * half_period)) * std::cos(M_PI * controlller_clock / half_period) * amplitude;

  // compute positional error
  P_error << p_target - p_measured, rotvec_err;

  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  // get H, g, A, b
  computeSolverParameters(q, dq, pinv_jacobian);

  // get M, h, g
  getDynamicsParameter(model, data, q, dq);

  // solve qp
  if (qp_initialized) {
    qp.update(qp_H, qp_g, qp_A, qp_b, std::nullopt, std::nullopt, std::nullopt);
  } else {
    qp.init(qp_H, qp_g, qp_A, qp_b, std::nullopt, std::nullopt, std::nullopt);
    qp_initialized = true;
  }
  qp.solve();

  torques = qp.results.x.bottomLeftCorner(7, 1);

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }
}

void WaypointController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void WaypointController::computeSolverParameters(const Eigen::Matrix<double, 7, 1>& q, 
                                           const Eigen::Matrix<double, 7, 1>& dq,
                                           const Eigen::Matrix<double, 7, 6>& pinv_jacobian) {
  Jddq_desired = ddP_cmd + tKp * P_error + tKd * (dP_target - jacobian * dq) - djacobian * dq;
  proj_mat = Eigen::MatrixXd::Identity(7, 7) - pinv_jacobian * jacobian;
  ddq_nominal = Kp * (q_nominal - q) - Kd * dq;

  qp_H.topLeftCorner(7, 7) = 2 * (jacobian.transpose() * jacobian + epsilon * proj_mat.transpose() * proj_mat);
  qp_g.topLeftCorner(7, 1) = -2 * (Jddq_desired.transpose() * jacobian + epsilon * ddq_nominal.transpose() * proj_mat.transpose() * proj_mat).transpose();
  qp_A << data.M, -Eigen::MatrixXd::Identity(7, 7);
  qp_b << -(data.nle - data.g);
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::WaypointController, controller_interface::ControllerBase)