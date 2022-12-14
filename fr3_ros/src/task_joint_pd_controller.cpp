#include <fr3_ros/task_joint_pd_controller.h>
#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>


#include <cmath>
#include <optional>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

namespace pin = pinocchio;

namespace fr3_ros {

bool TaskJointPDController::init(hardware_interface::RobotHW* robot_hw,
                                 ros::NodeHandle& node_handle) {
  
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("TaskJointPDController: Could not read parameter arm_id");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("TaskJointPDController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("TaskJointPDController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TaskJointPDController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TaskJointPDController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TaskJointPDController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "TaskJointPDController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "TaskJointPDController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "TaskJointPDController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR(
        "TaskJointPDController:  Invalid or no k_gain parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
    ROS_ERROR(
        "TaskJointPDController:  Invalid or no d_gain parameters provided, aborting "
        "controller init!");
    return false;
  }
  
  std::string urdf_filename;
  if (!node_handle.getParam("urdf_filename", urdf_filename)) {
    ROS_ERROR_STREAM("TaskJointPDController: Could not read parameter urdf_filename");
    return false;
  }

  // build pin_robot from urdf
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  control_log_publisher = registerLogPublisher(node_handle);
  marker_visulizer = new MarkerListVisualizer(node_handle, 2, 1000);

  return true;
}

void TaskJointPDController::starting(const ros::Time& /* time */) {
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

  // get Kp and Kd gains
  readJointPDGains(k_gains_, d_gains_, Kp, Kd);
  
  // initialize clock
  controlller_clock = 0.0;
}

void TaskJointPDController::update(const ros::Time& /*time*/, const ros::Duration& period) {
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
  ddP_cmd[1] = -(M_PI * M_PI / (half_period * half_period)) * std::sin(M_PI * controlller_clock / half_period) * amplitude;

  // compute positional error
  Eigen::Matrix<double, 6, 1> P_error;
  P_error << p_target - p_measured, rotvec_err;

  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  // compute joint target
  delta_q_target = pinv_jacobian * P_error;

  // compute joint torque
  ddq_cmd = pinv_jacobian * (ddP_cmd - djacobian * dq) - Kd * dq + Kp * pinv_jacobian * P_error + Kd * pinv_jacobian * dP_target;

  // get M, h, g
  getDynamicsParameter(model, data, q, dq);

  torques = data.M * ddq_cmd + (data.nle - data.g);

  // Saturate torque rate to avoid discontinuities
  torques << saturateTorqueRate(torques, tau_J_d);
  
  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }
  
  // log data
  logData.M = data.M;
  logData.C = data.C;
  logData.torque_cmd = torques;
  
  // publish the log data
  publishLogMsgs(&logData, &control_log_publisher);
  
  std::vector<Eigen::Matrix<double, 7, 1>> poses;
  std::vector<Eigen::Vector3d> colors;

  
  Eigen::VectorXd pose(7);
  Eigen::Vector3d color;

  color << 1,0,0;
  colors.push_back(color);

  color << 0,1,0;
  colors.push_back(color);

  pose << p_measured(0),p_measured(1),p_measured(2),0,0,0,1;
  poses.push_back(pose);
  pose << p_target(0),p_target(1),p_target(2),0,0,0,1;
  poses.push_back(pose);
  marker_visulizer->publish(poses, colors);
}

void TaskJointPDController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::TaskJointPDController, controller_interface::ControllerBase)