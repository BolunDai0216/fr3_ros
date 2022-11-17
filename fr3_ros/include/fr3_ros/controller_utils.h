#pragma once

#include <algorithm>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace fr3_ros {

Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                               const Eigen::Matrix<double, 7, 1>& tau_J_d);

inline void readJointPDGains(std::vector<double>& k_gains_,
                      std::vector<double>& d_gains_,
                      Eigen::Matrix<double, 7, 7>& Kp,
                      Eigen::Matrix<double, 7, 7>& Kd) {
  Eigen::Map<Eigen::Matrix<double, 7, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> d_gains_array(d_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
}

inline void readTaskPDGains(std::vector<double>& k_gains_,
                     std::vector<double>& d_gains_,
                     Eigen::Matrix<double, 6, 6>& Kp,
                     Eigen::Matrix<double, 6, 6>& Kd) {
  Eigen::Map<Eigen::Matrix<double, 6, 1>> k_gains_array(k_gains_.data());
  Eigen::Map<Eigen::Matrix<double, 6, 1>> d_gains_array(d_gains_.data());

  Kp = k_gains_array.array().matrix().asDiagonal();
  Kd = d_gains_array.array().matrix().asDiagonal();
}

}  // namespace fr3_ros