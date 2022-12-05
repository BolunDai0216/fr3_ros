#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <fr3_ros/controller_utils.h>
#include <Eigen/Dense>

namespace fr3_ros
{
class CartesianVelocityController
  : public controller_interface::MultiInterfaceController<
        franka_hw::FrankaVelocityCartesianInterface, franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface>
{
public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

private:
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  franka_hw::FrankaVelocityCartesianInterface* velocity_cartesian_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle> velocity_cartesian_handle_;
  ros::Duration elapsed_time_;

  // publisher object for the log messages
  ros::Publisher control_log_publisher;

  // for logging data
  LogDataType logData;
};

}  // namespace fr3_ros