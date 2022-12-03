#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <fr3_ros/controller_utils.h>
#include <Eigen/Dense>

namespace fr3_ros {

class JointVelocityController : public controller_interface::MultiInterfaceController<
                                       hardware_interface::VelocityJointInterface,
                                       franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  // publisher object for the log messages
  ros::Publisher control_log_publisher;

  // for logging data
  LogDataType logData;
};

}  // namespace fr3_ros