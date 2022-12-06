#include <fr3_ros/cartesian_velocity_controller.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>

namespace fr3_ros
{
bool CartesianVelocityController::init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle)
{
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id))
  {
    ROS_ERROR("CartesianVelocityController: Could not get parameter arm_id");
    return false;
  }

  velocity_cartesian_interface_ = robot_hardware->get<franka_hw::FrankaVelocityCartesianInterface>();
  if (velocity_cartesian_interface_ == nullptr)
  {
    ROS_ERROR(
        "CartesianVelocityController: Could not get Cartesian velocity interface from "
        "hardware");
    return false;
  }
  try
  {
    velocity_cartesian_handle_ = std::make_unique<franka_hw::FrankaCartesianVelocityHandle>(
        velocity_cartesian_interface_->getHandle(arm_id + "_robot"));
  }
  catch (const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianVelocityController: Error getting model interface from hardware");
    return false;
  }
  try
  {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  }
  catch (hardware_interface::HardwareInterfaceException& ex)
  {
    ROS_ERROR_STREAM("CartesianVelocityController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();

  if (state_interface == nullptr)
  {
    ROS_ERROR_STREAM("CartesianVelocityController: Error getting state interface from hardware");
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
            "CartesianVelocityController: Robot is not in the expected starting position for "
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

void CartesianVelocityController::starting(const ros::Time& /* time */)
{
  elapsed_time_ = ros::Duration(0.0);
}

void CartesianVelocityController::update(const ros::Time& /* time */, const ros::Duration& period)
{
  elapsed_time_ += period;

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_d(robot_state.dq_d.data());

  double T = 1.5;
  double amplitude = 0.1;
  double v_y = (2.0 * M_PI / T) * amplitude * std::sin((2.0 * M_PI / T) * elapsed_time_.toSec());

  // set end-effector velocity
  std::array<double, 6> command = { { 0.0, v_y, 0.0, 0.0, 0.0, 0.0 } };
  velocity_cartesian_handle_->setCommand(command);

  // // convert command to Eigen
  Eigen::Map<Eigen::Matrix<double, 6, 1>> P_dot_des(command.data());

  // // log data
  logData.q_dot = dq;
  logData.q_dot_des = dq_d;
  logData.P_dot = jacobian * dq;
  logData.P_dot_des = P_dot_des;

  // // publish the log data
  publishLogMsgs(&logData, &control_log_publisher);
}

void CartesianVelocityController::stopping(const ros::Time& /*time*/)
{
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::CartesianVelocityController, controller_interface::ControllerBase)