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

  // define rotation between R_end and R_start
  Eigen::Matrix<double, 3, 3> R_transform;
  R_transform << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

  // define terminal target for both position and orientation
  p_end << 0.4, 0.0, 0.3;
  R_end = R_transform * R_start;

  // duration of trajectory segment
  traj_duration = 10.0;

  // initialize clock
  elapsed_time_ = ros::Duration(0.0);
}

void KinematicCBFController::update(const ros::Time& /* time */, const ros::Duration& period)
{
  // update clock
  elapsed_time_ += period;

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // get end-effector Jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  // compute α, dα, ddα
  auto [alpha, dalpha, ddalpha] = getAlphas(elapsed_time_.toSec(), traj_duration);

  // end-effector position and velocity target
  p_target = alpha * (p_end - p_start);
  v_target = dalpha * (p_end - p_start);
  R_target = R_start;
  w_target << 0.0, 0.0, 0.0;

  P_error << p_target[0], p_target[1], p_target[2], 0.0, 0.0, 0.0;
  P_target << v_target[0], v_target[1], v_target[2], w_target[0], w_target[1], w_target[2];

  auto a = 10 * P_error + P_target;

  // compute pseudo-inverse of Jacobian
  franka_example_controllers::pseudoInverse(jacobian, pinv_jacobian);

  // compute commanded joint velocity
  dq_cmd = pinv_jacobian * a;

  for (size_t i = 0; i < 7; ++i)
  {
    velocity_joint_handles_[i].setCommand(dq_cmd[i]);
  }

  //   // log data
  //   logData.q = q;
  //   logData.q_dot = dq;
  //   logData.q_dot_des = omega * Eigen::MatrixXd::Ones(7, 1);

  //   // publish the log data
  //   publishLogMsgs(&logData, &control_log_publisher);
}

void KinematicCBFController::stopping(const ros::Time& /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::KinematicCBFController, controller_interface::ControllerBase)