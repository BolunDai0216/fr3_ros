#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace fr3_ros {

class RepetitiveCalibController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaPoseCartesianInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  franka_hw::FrankaPoseCartesianInterface* cartesian_pose_interface_;
  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  ros::Duration elapsed_time_;

  // end-effector pose
  std::array<double, 16> initial_pose_{};
  std::array<double, 16> target_pose_{};
  std::array<double, 16> current_pose_{};

  // clock only for controller
  double controlller_clock;

  // end-effector target
  double p_start_x, p_start_y, p_start_z;
  double p_target_x, p_target_y, p_target_z;
  double p_end_x, p_end_y, p_end_z;

  // waypoints
  std::array<Eigen::Matrix<double, 3, 1>, 4> waypoints;
  int waypoint_id;
  double traj_duration;
};

}  // namespace fr3_ros