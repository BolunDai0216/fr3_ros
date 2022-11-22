#include <controller_interface/controller_base.h>

#include <franka/gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <fr3_ros/repetitive_calib_controller.h>

#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <thread>
#include <cmath>

namespace fr3_ros {

bool RepetitiveCalibController::init(hardware_interface::RobotHW* robot_hardware,
                               ros::NodeHandle& node_handle) {
  cartesian_pose_interface_ = robot_hardware->get<franka_hw::FrankaPoseCartesianInterface>();

  if (cartesian_pose_interface_ == nullptr) {
    ROS_ERROR(
        "RepetitiveCalibController: Could not get Cartesian Pose "
        "interface from hardware");
    return false;
  }

  std::string arm_id;

  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("RepetitiveCalibController: Could not get parameter arm_id");
    return false;
  }

  try {
    cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
        cartesian_pose_interface_->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("RepetitiveCalibController: Exception getting Cartesian handle: " << e.what());
    return false;
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("RepetitiveCalibController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "RepetitiveCalibController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("RepetitiveCalibController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

void RepetitiveCalibController::starting(const ros::Time& /* time */) {
  // You can initialize any thing here.
  initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
  current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

  elapsed_time_ = ros::Duration(0.0);

  // set waypoints
  waypoints[0] << 0.407464, -0.276469, initial_pose_[14];
  waypoints[1] << 0.407464, -0.276469, 0.006;
  waypoints[2] << 0.407464, -0.276469, initial_pose_[14];
  waypoints[3] << initial_pose_[12], initial_pose_[13], initial_pose_[14];
  waypoint_id = 0;

  // get initial position
  p_start_x = initial_pose_[12];
  p_start_y = initial_pose_[13];
  p_start_z = initial_pose_[14];

  // set terminal position
  p_end_x = waypoints[waypoint_id](0, 0);
  p_end_y = waypoints[waypoint_id](1, 0);
  p_end_z = waypoints[waypoint_id](2, 0);

  traj_duration = 5.0;
}

void RepetitiveCalibController::update(const ros::Time& /* time */, const ros::Duration& period) {
  // update controller clock
  controlller_clock += period.toSec();

  // calculating alpha
  double alpha = std::sin(M_PI / 4 * (1 - std::cos((M_PI / traj_duration) * elapsed_time_.toSec())));

  // compute target pose
  target_pose_ = initial_pose_;
  target_pose_[12] = p_start_x + alpha * (p_end_x - p_start_x);
  target_pose_[13] = p_start_y + alpha * (p_end_y - p_start_y);
  target_pose_[14] = p_start_z + alpha * (p_end_z - p_start_z);

  // Sending the new position to robot Arm
  cartesian_pose_handle_->setCommand(target_pose_);
  
  // update target
  if (controlller_clock >= traj_duration + 1.0) {
    controlller_clock = 0.0;

    // set to next waypoint
    waypoint_id = (waypoint_id + 1) % 4;

    // get current pose
    current_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;

    // get initial position
    p_start_x = current_pose_[12];
    p_start_y = current_pose_[13];
    p_start_z = current_pose_[14];

    // set terminal position
    p_end_x = waypoints[waypoint_id](0, 0);
    p_end_y = waypoints[waypoint_id](1, 0);
    p_end_z = waypoints[waypoint_id](2, 0);
  }

}

}  // namespace fr3_ros

PLUGINLIB_EXPORT_CLASS(fr3_ros::RepetitiveCalibController,
                       controller_interface::ControllerBase)