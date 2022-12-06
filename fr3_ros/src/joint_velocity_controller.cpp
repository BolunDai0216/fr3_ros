#include <fr3_ros/joint_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace fr3_ros
{
bool JointVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr)
  {
    ROS_ERROR("JointVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("JointVelocityController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names))
  {
    ROS_ERROR("JointVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7)
  {
    ROS_ERROR_STREAM("JointVelocityController: Wrong number of joint names, got " << joint_names.size()
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
      ROS_ERROR_STREAM("JointVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // check if state_handle_ works
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();

  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("JointVelocityController: Error getting state interface from hardware");
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
            "JointVelocityController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch fr3_ros move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("JointVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  // define publisher
  control_log_publisher = registerLogPublisher(node_handle);

  return true;
}

void JointVelocityController::starting(const ros::Time& /* time */)
{
  elapsed_time_ = ros::Duration(0.0);
}

void JointVelocityController::update(const ros::Time& /* time */, const ros::Duration& period)
{
  elapsed_time_ += period;

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  ros::Duration half_cycle(5.0);
  double omega_max = 0.2;
  double cycle = std::floor(std::pow(
      -1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), half_cycle.toSec())) / half_cycle.toSec()));
  double omega = cycle * omega_max / 2.0 * (1.0 - std::cos(2.0 * M_PI / half_cycle.toSec() * elapsed_time_.toSec()));

  for (size_t i = 0; i < 7; ++i)
  {
    velocity_joint_handles_[i].setCommand(omega);
  }

  // log data
  logData.q = q;
  logData.q_dot = dq;
  logData.q_dot_des = omega * Eigen::MatrixXd::Ones(7, 1);

  // publish the log data
  publishLogMsgs(&logData, &control_log_publisher);
}

void JointVelocityController::stopping(const ros::Time& /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::JointVelocityController, controller_interface::ControllerBase)