#include <fr3_ros/kinematic_cbf_controller.h>
#include <fr3_ros/pinocchio_utils.h>
#include <franka_example_controllers/pseudo_inversion.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace fr3_ros
{
bool KinematicCBFController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr)
  {
    ROS_ERROR("KinematicCBFController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("KinematicCBFController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names))
  {
    ROS_ERROR("KinematicCBFController: Could not parse joint names");
  }
  if (joint_names.size() != 7)
  {
    ROS_ERROR_STREAM("KinematicCBFController: Wrong number of joint names, got " << joint_names.size()
                                                                                 << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i)
  {
    try
    {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    }
    catch (const hardware_interface::HardwareInterfaceException& ex)
    {
      ROS_ERROR_STREAM("KinematicCBFController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM("KinematicCBFController: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("KinematicCBFController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();

  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("KinematicCBFController: Error getting state interface from hardware");
    return false;
  }

  try
  {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));

    // nominal initial position
    std::array<double, 7> q_start{ { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } };

    for (size_t i = 0; i < q_start.size(); i++)
    {
      if (std::abs(state_handle_->getRobotState().q_d[i] - q_start[i]) > 0.1)
      {
        ROS_ERROR_STREAM(
            "KinematicCBFController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch fr3_ros move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("KinematicCBFController: Exception getting state handle: " << e.what());
    return false;
  }

  if (!node_handle.getParam("urdf_filename", urdf_filename))
  {
    ROS_ERROR_STREAM("KinematicCBFController: Could not read parameter urdf_filename");
    return false;
  }

  // build pin_robot from urdf
  pin::urdf::buildModel(urdf_filename, model);
  data = pin::Data(model);

  // define publisher
  control_log_publisher = registerLogPublisher(node_handle);

  return true;
}

void KinematicCBFController::starting(const ros::Time& /* time */)
{
  // get initial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_init(initial_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_init(initial_state.dq.data());

  // update data for initial configuration computation
  updatePinocchioModel(model, data, q_init, dq_init);

  // define end-effector frame id in pinocchio
  ee_frame_id = model.getFrameId("fr3_hand_tcp");

  // get current end-effector position and orientation
  p_start = data.oMf[ee_frame_id].translation();
  R_start = data.oMf[ee_frame_id].rotation();

  // define waypoints and set current target waypoint

  //   waypoints[0] << 0.6, 0.0, 0.3;
  //   waypoints[1] << 0.6, 0.0, 0.3;

  waypoints[0] << 0.4, 0.0, 0.7;
  waypoints[1] << 0.7, 0.0, 0.5;
  waypoints[2] << 0.7, 0.0, 0.5;
  waypoints[3] << 0.5, 0.0, 0.5;
  waypoints[4] << 0.35, 0.0, 0.5;

  //   waypoints[0] << 0.4, 0.0, 0.7;
  //   waypoints[1] << 0.6, 0.0, 0.3;
  //   waypoints[2] << 0.6, 0.0, 0.3;
  //   waypoints[3] << 0.5, 0.0, 0.3;
  //   waypoints[4] << 0.3, 0.0, 0.5;

  waypoint_id = 0;

  // define rotation between R_end and R_start
  Eigen::Matrix<double, 3, 3> R_transform;
  R_transform << 0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0;
  //   R_transform << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  waypoint_rotmats[0] = R_transform * R_start;
  waypoint_rotmats[1] = R_transform * R_start;
  waypoint_rotmats[2] = R_transform * R_start;
  waypoint_rotmats[3] = R_transform * R_start;
  waypoint_rotmats[4] = R_start;

  // define CBF types along the trajectory
  cbf_types[0] = "x";
  cbf_types[1] = "x";
  cbf_types[2] = "z";
  cbf_types[3] = "z";
  cbf_types[4] = "x";

  // define terminal target for both position and orientation
  resetTarget();

  // duration of trajectory segment
  traj_duration = 8.0;

  // initialize clock
  elapsed_time_ = ros::Duration(0.0);

  // initialize qp parameters with zeros
  qp_H = Eigen::MatrixXd::Zero(7, 7);
  qp_g = Eigen::MatrixXd::Zero(7, 1);
  qp_C = Eigen::MatrixXd::Zero(1, 7);
  qp_lb = Eigen::MatrixXd::Zero(1, 1);
  qp_ub = Eigen::MatrixXd::Zero(1, 1);

  // set nominal joint configuration
  q_nominal = q_init;
}

void KinematicCBFController::update(const ros::Time& /* time */, const ros::Duration& period)
{
  // update clock
  elapsed_time_ += period;

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // update pinocchio
  updatePinocchioModel(model, data, q, dq);

  // get end-effector jacobian and its time derivative
  pin::getFrameJacobian(model, data, ee_frame_id, pin::LOCAL_WORLD_ALIGNED, jacobian);

  // get current end-effector position and orientation
  p_current = data.oMf[ee_frame_id].translation();
  R_current = data.oMf[ee_frame_id].rotation();

  // move to next target
  if (elapsed_time_.toSec() >= traj_duration + 3.0)
  {
    resetTarget();
  }

  // compute α, dα, ddα
  auto [alpha, dalpha, ddalpha] = getAlphas(elapsed_time_.toSec(), traj_duration);

  // end-effector position and velocity target
  p_target = alpha * (p_end - p_start) + p_start;
  v_target = dalpha * (p_end - p_start);

  // end-effector orientational targets
  Eigen::Matrix<double, 3, 3> R_error_path = R_end * R_start.transpose();
  Eigen::AngleAxisd AngleAxisErrPath(R_error_path);
  Eigen::AngleAxisd TargetAngleAxisErr(alpha * AngleAxisErrPath.angle(), AngleAxisErrPath.axis());

  R_target = TargetAngleAxisErr.toRotationMatrix() * R_start;
  w_target = alpha * TargetAngleAxisErr.axis() * TargetAngleAxisErr.angle();

  //   R_target = R_start;
  //   w_target << 0.0, 0.0, 0.0;

  // compute positional errors
  auto rotvec_err = computeRotVecError(R_target, R_current);

  P_error << p_target - p_current, rotvec_err;
  dP_target << v_target, w_target;

  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  // get qp_H, qp_g, qp_C, qp_lb
  computeSolverParameters(q, dq);

  // solve qp
  if (qp_initialized)
  {
    qp.update(qp_H, qp_g, std::nullopt, std::nullopt, qp_C, qp_lb, qp_ub);
    // qp.update(qp_H, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  }
  else
  {
    qp.init(qp_H, qp_g, std::nullopt, std::nullopt, qp_C, qp_lb, qp_ub);
    // qp.init(qp_H, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
    qp_initialized = true;
  }
  qp.solve();

  // get commanded joint velocity from solver
  dq_cmd = qp.results.x;

  for (size_t i = 0; i < 7; ++i)
  {
    velocity_joint_handles_[i].setCommand(dq_cmd[i]);
  }

  //   ROS_INFO_STREAM("P_EE: " << p_current.transpose());
  //   ROS_INFO_STREAM("p_error: " << (p_target - p_current).transpose());
  //   ROS_INFO_STREAM("v_target: " << v_target.transpose());
}

void KinematicCBFController::stopping(const ros::Time& /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void KinematicCBFController::computeSolverParameters(const Eigen::Matrix<double, 7, 1>& q,
                                                     const Eigen::Matrix<double, 7, 1>& dq)
{
  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  Jdq_desired = 10.0 * P_error + dP_target;
  proj_mat = Eigen::MatrixXd::Identity(7, 7) - pinv_jacobian * jacobian;
  dq_nominal = 1.0 * (q_nominal - q);

  qp_H.topLeftCorner(7, 7) = 2 * (jacobian.transpose() * jacobian + epsilon * proj_mat.transpose() * proj_mat);
  qp_g.topLeftCorner(7, 1) =
      -2 * (Jdq_desired.transpose() * jacobian + epsilon * dq_nominal.transpose() * proj_mat.transpose() * proj_mat)
               .transpose();

  if (cbf_type == "x")
  {
    qp_C.topLeftCorner(1, 7) = -jacobian.row(0);
    qp_lb << -1.0 * (0.5 - p_current[0, 0]);
    qp_ub << 10000.0;
  }

  if (cbf_type == "z")
  {
    qp_C.topLeftCorner(1, 7) = -jacobian.row(2);
    qp_lb << -1.0 * (1.0 - p_current[2, 0]);
    qp_ub << 10000.0;
  }

  ROS_INFO_STREAM("gap: " << qp_C * dq - qp_lb);
}

void KinematicCBFController::resetTarget(void)
{
  // get current end-effector position
  p_start = data.oMf[ee_frame_id].translation();
  R_start = data.oMf[ee_frame_id].rotation();

  // define terminal target for both position and orientation
  p_end = waypoints[waypoint_id];
  R_end = waypoint_rotmats[waypoint_id];

  ROS_INFO_STREAM("Target P_EE: " << p_end.transpose());

  // reset clock
  elapsed_time_ = ros::Duration(0.0);

  // set CBF type
  cbf_type = cbf_types[waypoint_id];

  // update waypoint ID
  waypoint_id = (waypoint_id + 1) % 5;
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::KinematicCBFController, controller_interface::ControllerBase)