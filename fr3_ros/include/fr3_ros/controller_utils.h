#pragma once

#include <algorithm>
#include <vector>
#include <tuple>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "fr3_ros/controlLogs.h"
#include "ros/ros.h"

namespace fr3_ros {

struct LogDataType{
  Eigen::Matrix<double, 7, 1>  q;
  Eigen::Matrix<double, 7, 1> q_dot;
  Eigen::Matrix<double, 7, 1> q_des;
  Eigen::Matrix<double, 7, 1> q_dot_des;
  Eigen::Matrix<double, 7, 7> M;
  Eigen::Matrix<double, 7, 7> C;
  Eigen::Matrix<double, 7, 1> G;
  Eigen::Matrix<double, 6, 7> J;
  Eigen::Matrix<double, 6, 7> J_dot;
  Eigen::Matrix<double, 6, 7> J_aux;
  Eigen::Matrix<double, 7, 7> M_aux;
  Eigen::Matrix<double, 7, 1> torque_cmd;
};

ros::Publisher registerLogPublisher(ros::NodeHandle& node_handle);

void publishLogMsgs(LogDataType *data, ros::Publisher *pub);


Eigen::Matrix<double, 7, 1> saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                               const Eigen::Matrix<double, 7, 1>& tau_J_d);
                                  
Eigen::Vector3d computeRotVecError(const Eigen::Matrix<double, 3, 3>& R_target,
                                   const Eigen::Matrix<double, 3, 3>& R_measured);
                            
std::tuple<double, double, double> getAlphas(const double& t, const double& T); 

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