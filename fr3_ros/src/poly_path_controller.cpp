#include <fr3_ros/poly_path_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <fr3_ros/pinocchio_utils.h>
#include <fr3_ros/controller_utils.h>
#include <fr3_ros/traj_gen.h>

#include <cmath>
#include <optional>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/utils/random_qp_problems.hpp>
#include <Eigen/SVD>

namespace pin = pinocchio;

namespace fr3_ros {

bool PolyPathController::init(hardware_interface::RobotHW* robot_hw,
                              ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("PolyPathController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PolyPathController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PolyPathController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PolyPathController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PolyPathController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PolyPathController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PolyPathController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PolyPathController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PolyPathController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "PolyPathController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "PolyPathController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("tk_gains", tk_gains_) || tk_gains_.size() != 6) {
    ROS_ERROR(
        "PolyPathController:  Invalid or no tk_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("td_gains", td_gains_) || td_gains_.size() != 6) {
    ROS_ERROR(
        "PolyPathController:  Invalid or no td_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  if (!node_handle.getParam("urdf_filename", urdf_filename)) {
    ROS_ERROR_STREAM("PolyPathController: Could not read parameter urdf_filename");
    return false;
  }

  if (!node_handle.getParam("epsilon", epsilon)) {
    ROS_ERROR_STREAM("PolyPathController: Could not read parameter epsilon");
    return false;
  }

  // build pin_robot from urdf
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  // define publisher
  control_log_publisher = registerLogPublisher(node_handle);

  return true;
}

void PolyPathController::starting(const ros::Time& /* time */) {
  // get intial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_init(initial_state.dq.data());

  // update data for initial configuration computation
  updatePinocchioModel(model, data, q_init, dq_init);

  // define end-effector frame id in pinocchio
  ee_frame_id = model.getFrameId("fr3_hand_tcp");

  // define duration of each trajctory segment
  traj_duration = 10.0;

  // get current end-effector position and orientation
  p_start = data.oMf[ee_frame_id].translation();
  R_start = data.oMf[ee_frame_id].rotation();

  // initialize dP target and commanded ddP
  dP_target << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  ddP_cmd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  // define waypoints and set current target waypoint
  waypoints[0] << 0.4, 0.4, 0.2;
  waypoints[1] << 0.3, 0.0, 0.5;
  waypoints[2] << 0.4, -0.4, 0.2;
  waypoints[3] << 0.3, 0.0, 0.5;
  waypoint_id = 0;

  // define terminal target for both position and orientation
  p_end = waypoints[waypoint_id];
  R_end = R_start;

  // initialize qp parameters with zeros
  qp_H = Eigen::MatrixXd::Zero(14, 14);
  qp_g = Eigen::MatrixXd::Zero(14, 1);
  qp_A = Eigen::MatrixXd::Zero(7, 14);
  qp_b = Eigen::MatrixXd::Zero(7, 1);

  // set nominal joint configuration
  q_nominal = q_init;

  // get Kp and Kd gains
  readJointPDGains(k_gains_, d_gains_, Kp, Kd);
  readTaskPDGains(tk_gains_, td_gains_, tKp, tKd);

  // compute coefficient for polynomial path
  computePolyJerk(p_start, p_end, traj_duration, poly_x, poly_y, poly_z);
  
  // initialize clock
  controlller_clock = 0.0;
}

void PolyPathController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());

  // measure end-effector position and orientation
  updatePinocchioModel(model, data, q, dq);
  p_measured = data.oMf[ee_frame_id].translation();
  R_measured = data.oMf[ee_frame_id].rotation();
  
  // move to next target
  if (controlller_clock >= traj_duration + 1.0) {
    resetTarget();
  }

  // get end-effector jacobian and its time derivative
  pin::getFrameJacobian(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, jacobian);
  pin::getFrameJacobianTimeVariation(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, djacobian);

  // compute p, v, a, w, dw targets and rotvec_err
  computeEndEffectorTarget(controlller_clock, traj_duration);

  // set target configuration at this time step
  P_error << p_target - p_measured, rotvec_err;
  dP_target << v_target, w_target;
  ddP_cmd << a_target, dw_target;

  // get qp_H, qp_g, qp_A, qp_b
  computeSolverParameters(q, dq);

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

  // get solved torques from qp solution
  torques = qp.results.x.bottomLeftCorner(7, 1);

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  // compute singular values
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd;
  // svd.compute(jacobian, Eigen::ComputeThinV | Eigen::ComputeThinU);
  // ROS_INFO_STREAM("Its singular values are: \n" << svd.singularValues());

  // ROS_INFO_STREAM("P_error: " << P_error.transpose());
  // ROS_INFO_STREAM("p_measured: " << p_measured.transpose());

  // log data
  logData.M = data.M;
  logData.C = data.C;
  logData.torque_cmd = torques;
  
  // publish the log data
  publishLogMsgs(&logData, &control_log_publisher);
}

void PolyPathController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void PolyPathController::computeSolverParameters(const Eigen::Matrix<double, 7, 1>& q, 
                                                 const Eigen::Matrix<double, 7, 1>& dq) {
  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);
                                                
  Jddq_desired = ddP_cmd + tKp * P_error + tKd * (dP_target - jacobian * dq) - djacobian * dq;
  proj_mat = Eigen::MatrixXd::Identity(7, 7) - pinv_jacobian * jacobian;
  ddq_nominal = Kp * (q_nominal - q) - Kd * dq;

  qp_H.topLeftCorner(7, 7) = 2 * (jacobian.transpose() * jacobian + epsilon * proj_mat.transpose() * proj_mat);
  qp_g.topLeftCorner(7, 1) = -2 * (Jddq_desired.transpose() * jacobian + epsilon * ddq_nominal.transpose() * proj_mat.transpose() * proj_mat).transpose();
  qp_A << data.M, -Eigen::MatrixXd::Identity(7, 7);
  qp_b << -(data.nle - data.g);
}

void PolyPathController::resetTarget(void) {
  // get current end-effector position
  p_start = data.oMf[ee_frame_id].translation();

  // set waypoint to next one
  waypoint_id = (waypoint_id + 1) % 4;

  // define position terminal target, orientation target is unchanged
  p_end = waypoints[waypoint_id];

  // compute coefficient for polynomial path
  computePolyJerk(p_start, p_end, traj_duration, poly_x, poly_y, poly_z);

  // reset controller_clock
  controlller_clock = 0.0;
}

void PolyPathController::computeEndEffectorTarget(const double& controlller_clock, const double& traj_duration) {
  // compute p, v, a targets
  computePolyJerkTargets(controlller_clock, traj_duration, poly_x, poly_y, poly_z, p_target, v_target, a_target);
  
  // compute orientational targets
  R_target = R_start;
  rotvec_err = computeRotVecError(R_target, R_measured);
  w_target = Eigen::MatrixXd::Zero(3, 1);
  dw_target = Eigen::MatrixXd::Zero(3, 1);
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::PolyPathController, controller_interface::ControllerBase)